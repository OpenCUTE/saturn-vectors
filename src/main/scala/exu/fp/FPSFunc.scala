package saturn.exu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import chisel3.util.experimental.decode._
import saturn.common._
import saturn.insns._

class Exp2LUT64 extends Module {
  val io = IO(new Bundle {
    val idx  = Input(UInt(6.W))
    val y0   = Output(UInt(24.W)) // Q1.23
    val y1   = Output(UInt(24.W))
  })

  val table = VecInit((0 until 65).map { i =>
    val v = math.pow(2.0, i.toDouble / 64.0)
    (v * (math.pow(2.0, 23))).toInt.U(24.W)
  })

  io.y0 := table(io.idx)
  io.y1 := table(io.idx + 1.U)
}

class LinearInterp extends Module {
  val io = IO(new Bundle {
    val y0    = Input(UInt(24.W)) // Q1.23
    val y1    = Input(UInt(24.W))
    val alpha = Input(UInt(17.W)) // [0,1)
    val out   = Output(UInt(24.W))
  })

  val diff = io.y1 - io.y0                // Q1.23
  val prod = diff * io.alpha              // Q1.23 * Q0.17 = Q1.40
  val interp = io.y0 + (prod >> 17)        // back to Q1.23

  io.out := interp
}

class FP32Exp2Interp extends Module {
  val io = IO(new Bundle {
    val in  = Input(UInt(32.W))
    val out = Output(UInt(32.W))
  })

  val sign = io.in(31)
  val exp  = io.in(30,23)
  val frac = io.in(22,0)

  // ---------- FP32 → Q8.16 ----------
  val mant = Cat(1.U(1.W), frac)                 // 1.xxx
  val e = exp.asSInt - 127.S
  val fixed = Mux(e > 0.S, (mant.asSInt << e.asUInt)(31,0), (mant.asSInt >> (-e).asUInt)(31,0))

  val I = fixed(31,23).asSInt
  val F = fixed(22,0)

  // ---------- 插值索引 ----------
  val Fh = F(22,17)  // 6 bit
  val Fl = F(16,0)    // 17 bit

  val lut = Module(new Exp2LUT64)
  lut.io.idx := Fh

  val interp = Module(new LinearInterp)
  interp.io.y0    := lut.io.y0
  interp.io.y1    := lut.io.y1
  interp.io.alpha := Fl

  // ---------- FP32 pack ----------
  val outExp = (127.S + I).asUInt

  val outMant = (interp.io.out * lut.io.y0) >> 23 // Q1.23 * Q1.23 >> 23 = Q1.23

  io.out := Cat(
    0.U(1.W),
    outExp(7,0),
    outMant(22,0)
  )
}

class VEXP2FAKE(implicit p: Parameters) extends FPUModule()(p) {
  val io = IO(new Bundle {
    val in_valid = Input(Bool())
    val rvs2_input = Input(UInt(32.W))
    // val eew = Input(UInt(2.W))
    val out_valid = Output(Bool())
    val out = Output(UInt(32.W))
    val exc = Output(UInt(5.W))
  })

  val rvs2_bits = io.rvs2_input

  val fType = FType.S
  val classify = fType.classify(fType.recode(rvs2_bits(fType.ieeeWidth-1,0)))

  val dz = WireInit(false.B)
  val nv = WireInit(false.B)
  val of = WireInit(false.B)
  val nx = WireInit(false.B)

  val ret = Wire(UInt(32.W))
  ret := 0.U // it should not be possible to fall into this case
  when (classify(0)) { // -inf
    ret := 0.U
  } .elsewhen (classify(7)) { // +inf
    ret := 0.U(1.W) ## ~(0.U((fType.exp).W)) ## 0.U((fType.sig-1).W)
  } .elsewhen (classify(3)) { // -0
    ret := 0.U(2.W) ## ~(0.U((fType.exp-1).W)) ## 0.U((fType.sig-1).W)
    dz := true.B
  } .elsewhen (classify(4)) { // +0
    ret := 0.U(2.W) ## ~(0.U((fType.exp-1).W)) ## 0.U((fType.sig-1).W)
    dz := true.B
  } .elsewhen (classify(8)) { // sNaN
    ret := fType.ieeeQNaN
    nv := true.B
  } .elsewhen (classify(9)) { // qNaN
    ret := fType.ieeeQNaN
  } .otherwise {
    
    ret := ~rvs2_bits

  }

  io.out := ret
  io.out_valid := io.in_valid
  io.exc := nv ## dz ## of ## false.B ## nx
}

case object FPSFuncFactory extends FunctionalUnitFactory {
  def insns = Seq(
    FEXP2_V
  ).map(_.restrictSEW(1,2,3)).flatten.map(_.pipelined(1))

  def generate(implicit p: Parameters) = new FPSFuncPipe()(p)
}

class FPSFuncPipe(implicit p : Parameters) extends PipelinedFunctionalUnit(1)(p) with HasFPUParameters {
  val supported_insns = FPSFuncFactory.insns
  io.set_vxsat := false.B

  val nTandemFMA = dLenB / 4

  val valid = RegInit(false.B)
  val op = Reg(new ExecuteMicroOpWithData(1))


  when(io.iss.valid) {
    op := io.iss.op
    valid := true.B
  }.elsewhen(io.write.fire) {
    valid := false.B
  }


  val vec_rvs1 = op.rvs1_data.asTypeOf(Vec(nTandemFMA, UInt(32.W)))
  val vec_rvs2 = op.rvs2_data.asTypeOf(Vec(nTandemFMA, UInt(32.W)))
  // val vec_rvd = io.pipe(0).bits.rvd_data.asTypeOf(Vec(nTandemFMA, UInt(64.W)))

//   val vfclass_inst = op.opff6.isOneOf(OPFFunct6.funary1) && op.rs1 === 16.U


//   val ctrl = new VectorDecoder(
//     op,
//     supported_insns)

  val exp2_op = op.rs1 === 8.U

  val outexc = Wire(Vec(nTandemFMA, UInt(5.W)))
  val out = Wire(Vec(nTandemFMA, UInt(32.W)))
  val exp2outValid = Wire(Vec(nTandemFMA, Bool()))

  val pipe_out = (0 until nTandemFMA).map {i =>

    val exp2 = Module(new VEXP2FAKE)

    val rvs2_bits = vec_rvs2(i)

    val iss_fire_pipe = Reg(Bool())
    iss_fire_pipe := io.iss.valid

    exp2.io.in_valid := iss_fire_pipe && exp2_op
    exp2.io.rvs2_input := Mux(valid && exp2_op, rvs2_bits, 0.U)

    val exp2_out_valid = exp2.io.out_valid
    val exp2_out = exp2.io.out

    // val elemout = Mux1H(
    //   Seq(vfclass_inst, vfrsqrt7_inst, vfrec7_inst, divsqrt_valid),
    //   Seq(gen_vfclass, recSqrt7.io.out, rec7.io.out, divsqrt_reg)
    // )(63,0)

    outexc(i) := Mux(exp2_out_valid, exp2.io.exc,
          0.U)
    exp2outValid(i) := exp2_out_valid
    out(i) := exp2_out
  }

  io.set_fflags.valid := exp2outValid.asUInt.andR
  io.set_fflags.bits := outexc.reduce(_|_)

  io.scalar_write.valid := false.B
  io.scalar_write.bits := DontCare

  io.write.valid := io.pipe(depth-1).valid
  io.write.bits.eg := op.wvd_eg
  io.write.bits.mask := FillInterleaved(8, op.wmask)
  io.write.bits.data := out.asUInt
  io.stall := valid
}
