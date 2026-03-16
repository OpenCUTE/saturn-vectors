package saturn.exu

import chisel3._
import circt.stage.ChiselStage
import chisel3.util._

// filter -> RangeReduce -> poly -> compose
object SFUOp {
  val EXP2  = 0.U(3.W)
  val LOG2  = 1.U(3.W)
  val RCP   = 2.U(3.W)
  val SQRT  = 3.U(3.W)
  val RSQRT = 4.U(3.W)
  val SIN   = 5.U(3.W)
  val COS   = 6.U(3.W)
}

// just for readability, do not change values
object SFUConfig {
  val c0Width            = 26
  val c1Width            = 16
  val c2Width            = 12
  val squarerOutputWidth = 15
}

// constants for special values, changes may crash the hardware behavior
case class FunctionParams(
  m:      Int,
  c0Sign: Int,
  c1Sign: Int,
  c2Sign: Int,
  c0Exp:  Int,
  c1Exp:  Int,
  c2Exp:  Int
) {
  val c0RealExp:    Int = c0Exp - SFUConfig.c0Width + 1
  val c1RealExp:    Int = c1Exp - SFUConfig.c1Width + 1
  val c2RealExp:    Int = c2Exp - SFUConfig.c2Width + 1
  val c1XlRealExp:  Int = c1RealExp - 23
  val c2Xl2RealExp: Int = c2RealExp - 2 * m - SFUConfig.squarerOutputWidth
  val shift0:       Int = 2 * (23 - m) - SFUConfig.squarerOutputWidth
  val shift1:       Int = c0RealExp - c1XlRealExp
  val shift2:       Int = c0RealExp - c2Xl2RealExp
}

object Function {
  val EXP2  = FunctionParams(6, 1,  1,  1, 1,  1, -1)
  val SIN   = FunctionParams(6, 1,  1,  -1, 0,  1, 1)

  def getShift0(op: UInt): UInt = {
    MuxLookup(op, 0.U(5.W)) (Seq(
      SFUOp.EXP2  -> EXP2.shift0.U(5.W),
      SFUOp.SIN   -> SIN.shift0.U(5.W),
      SFUOp.COS   -> SIN.shift0.U(5.W)
    ))
  }

  def getShift1(op: UInt): UInt = {
    MuxLookup(op, 0.U(5.W)) (Seq(
      SFUOp.EXP2  -> EXP2.shift1.U(5.W),
      SFUOp.SIN   -> SIN.shift1.U(5.W),
      SFUOp.COS   -> SIN.shift1.U(5.W)
    ))
  }
  def getShift2(op: UInt): UInt = {
    MuxLookup(op, 0.U(5.W)) (Seq(
      SFUOp.EXP2  -> EXP2.shift2.U(5.W),
      SFUOp.SIN   -> SIN.shift2.U(5.W),
      SFUOp.COS   -> SIN.shift2.U(5.W)
    ))
  }

}

object SFUParameters {
  val MIN_INPUT_EXP2 = "hC3000000".U(32.W) // -128.0
  val MAX_INPUT_EXP2 = "h43000000".U(32.W) // 128.0

  val POS_ZERO       = "h00000000".U(32.W)
  val POS_ONE        = "h3F800000".U(32.W)
  val POS_INF        = "h7F800000".U(32.W)

  val NEG_ZERO       = "h80000000".U(32.W)
  val NEG_ONE        = "hBF800000".U(32.W)
  val NEG_INF        = "hFF800000".U(32.W)

  val NAN            = "h7FFFFFFF".U(32.W)
}

object SFUUtils {
  implicit class DecoupledPipe[T <: Data](val decoupledBundle: DecoupledIO[T]) extends AnyVal {
    def handshakePipeIf(en: Boolean): DecoupledIO[T] = {
      if (en) {
        val out    = Wire(Decoupled(chiselTypeOf(decoupledBundle.bits)))
        val rValid = RegInit(false.B)
        val rBits  = Reg(chiselTypeOf(decoupledBundle.bits))
        decoupledBundle.ready  := !rValid || out.ready
        out.valid              := rValid
        out.bits               := rBits
        when(decoupledBundle.fire) {
          rBits  := decoupledBundle.bits
          rValid := true.B
        } .elsewhen(out.fire) {
          rValid := false.B
        }
        out
      } else {
        decoupledBundle
      }
    }
  }
}

import SFUUtils._

class SFUInput extends Bundle {
  val x  = UInt(32.W)
  val op = UInt(3.W)
}

class FilterToRangeReduce extends Bundle {
  val sign      = UInt(1.W)
  val exponent  = UInt(8.W)
  val mantissa  = UInt(23.W)
  val op        = UInt(3.W)
  val bypass    = Bool()
  val bypassVal = UInt(32.W)
}

class RangeReduceToLookup extends Bundle {
  val index     = UInt(7.W)
  val xl        = UInt(17.W)
  val sign      = UInt(1.W)
  val exp       = SInt(8.W)
  val op        = UInt(3.W)
  val bypass    = Bool()
  val bypassVal = UInt(32.W)
}

class LookupToPoly extends Bundle {
  val c0        = SInt(27.W)
  val c1        = SInt(17.W)
  val c2        = SInt(13.W)
  val xl        = UInt(17.W)
  val sign      = UInt(1.W)
  val exp       = SInt(8.W)
  val op        = UInt(3.W)
  val bypass    = Bool()
  val bypassVal = UInt(32.W)
}

class PolyToCompose extends Bundle {
  val polyResult = UInt(27.W)
  val sign       = UInt(1.W)
  val exp        = SInt(8.W)
  val op         = UInt(3.W)
  val bypass     = Bool()
  val bypassVal  = UInt(32.W)
}

class SFUOutput extends Bundle {
  val result = UInt(32.W)
}


class Filter extends Module {
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new SFUInput))
    val out = Decoupled(new FilterToRangeReduce)
  })
 
  val s = io.in.bits.x(31)
  val e = io.in.bits.x(30, 23)
  val m = io.in.bits.x(22, 0)
 
  val isZero   =  e === 0.U // subnormal numbers are treated as zero
  val isInf    = (e === "hFF".U) && (m === 0.U)
  val isNaN    = (e === "hFF".U) && (m =/= 0.U)
  val isNeg    =  s === 1.U
 
  val tooBig = (!s) && (e >= 134.U)
  val tooNeg =   s  && (e >= 134.U)

  val bypass    = Wire(Bool())
  val bypassVal = Wire(UInt(32.W))

  when(io.in.bits.op === SFUOp.EXP2) {
    bypass    := isZero || isInf || isNaN || tooBig || tooNeg
    bypassVal := MuxCase(SFUParameters.POS_ZERO, Seq(
      isZero  -> SFUParameters.POS_ONE,
      isInf   -> Mux(isNeg, SFUParameters.POS_ZERO, SFUParameters.POS_INF),
      isNaN   -> SFUParameters.NAN,
      tooBig  -> SFUParameters.POS_INF,
      tooNeg  -> SFUParameters.POS_ZERO
    ))
  }.elsewhen(io.in.bits.op === SFUOp.SIN) {
    bypass    := isZero || isInf || isNaN
    bypassVal := MuxCase(SFUParameters.POS_ZERO, Seq(
      isZero -> Mux(isNeg, SFUParameters.NEG_ZERO, SFUParameters.POS_ZERO),
      isInf  -> SFUParameters.NAN,
      isNaN  -> SFUParameters.NAN
    ))
  }.elsewhen(io.in.bits.op === SFUOp.COS) {
    bypass    := isZero || isInf || isNaN
    bypassVal := MuxCase(SFUParameters.POS_ONE, Seq(
      isZero -> SFUParameters.POS_ONE,
      isInf  -> SFUParameters.NAN,
      isNaN  -> SFUParameters.NAN
    ))
  }.otherwise {
    bypass    := false.B
    bypassVal := SFUParameters.POS_ZERO
  }
 
  val s1 = Wire(Decoupled(new FilterToRangeReduce))
  val s1Pipe = s1.handshakePipeIf(true)
 
  s1.valid           := io.in.valid
  s1.bits.sign       := s
  s1.bits.exponent   := e
  s1.bits.mantissa   := m
  s1.bits.op         := io.in.bits.op
  s1.bits.bypass     := bypass
  s1.bits.bypassVal  := bypassVal
  io.in.ready        := s1.ready
 
  io.out <> s1Pipe
}

class RangeReduce extends Module {
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new FilterToRangeReduce))
    val out = Decoupled(new RangeReduceToLookup)
  })

  val op       = io.in.bits.op
  val sign     = io.in.bits.sign.asBool
  val rawExp   = io.in.bits.exponent
  val mantissa = io.in.bits.mantissa
  val sig      = Cat(1.U(1.W), mantissa)

  // exp2 specific range reduction
  val expSigned     = (rawExp.zext - 127.S).asSInt
  val sigExtended   = Cat(0.U(7.W), sig)
  val shift         = expSigned
  val sigShifted    = Mux(shift >= 0.S, sigExtended << shift.asUInt, sigExtended >> (-shift).asUInt)
  val intPart       = sigShifted(30, 23)
  val fracPart      = sigShifted(22, 0)
  val fracPartInv   =  ~fracPart
  val isFracZero    = fracPart === 0.U
  val intPartFloor  = Mux(sign && (!isFracZero), intPart + 1.U    , intPart)
  val fracPartFloor = Mux(sign && (!isFracZero), fracPartInv + 1.U, fracPart)

  // sin/cos specific quadrant calculation
  val quadrant = intPart(1, 0)
  val signSin  = sign.asUInt ^ quadrant(1)
  val signCos  = quadrant(1) ^ quadrant(0)

  val fracSin  = Mux(quadrant(0), fracPartInv , fracPart)
  val fracCos  = Mux(quadrant(0), fracPart    , fracPartInv)

  val signFinal = MuxLookup(op, sign.asUInt) (Seq(
    SFUOp.SIN -> signSin,
    SFUOp.COS -> signCos
  ))

  val exp = MuxLookup(op, 0.S(8.W)) (Seq(
    SFUOp.EXP2  -> Mux(sign, -(intPartFloor.asSInt), intPartFloor.asSInt),
    SFUOp.SIN   -> 0.S(8.W),
    SFUOp.COS   -> 0.S(8.W)
  ))

  val index = MuxLookup(op, 0.U(7.W)) (Seq(
    SFUOp.EXP2  -> Cat(0.U(1.W), fracPartFloor(22, 17)),
    SFUOp.SIN   -> Cat(0.U(1.W), fracSin(22, 17)),
    SFUOp.COS   -> Cat(0.U(1.W), fracCos(22, 17))
  ))

  val xl = MuxLookup(op, 0.U(32.W)) (Seq(
    SFUOp.EXP2  -> fracPartFloor(16, 0),
    SFUOp.SIN   -> fracSin(16, 0),
    SFUOp.COS   -> fracCos(16, 0)
  ))
 
  val s1     = Wire(Decoupled(new RangeReduceToLookup))
  val s1Pipe = s1.handshakePipeIf(true)
  io.in.ready        := s1.ready
  s1.valid           := io.in.valid
  s1.bits.index      := index
  s1.bits.xl         := xl
  s1.bits.sign       := signFinal
  s1.bits.exp        := exp
  s1.bits.op         := op
  s1.bits.bypass     := io.in.bits.bypass
  s1.bits.bypassVal  := io.in.bits.bypassVal
 
  io.out <> s1Pipe
}

class LookupTable extends Module {
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new RangeReduceToLookup))
    val out = Decoupled(new LookupToPoly)
  })
 
  class LUTEntry extends Bundle {
    val c0 = SInt(27.W)
    val c1 = SInt(17.W)
    val c2 = SInt(13.W)
  }
 
  def loadLUT(filename: String, c0Sign: Int, c1Sign: Int, c2Sign: Int): Seq[LUTEntry] = {
    val lines = scala.io.Source.fromFile(filename).getLines().toSeq
    lines.map { line =>
      val parts = line.trim.split("\\s+")
      val c0Hex = parts(0)
      val c1Hex = parts(1)
      val c2Hex = parts(2)

      val c0Unsigned = BigInt(c0Hex, 16)
      val c1Unsigned = BigInt(c1Hex, 16)
      val c2Unsigned = BigInt(c2Hex, 16)

      val c0Signed = if (c0Sign == -1) -c0Unsigned else c0Unsigned
      val c1Signed = if (c1Sign == -1) -c1Unsigned else c1Unsigned
      val c2Signed = if (c2Sign == -1) -c2Unsigned else c2Unsigned
 
      val entry = Wire(new LUTEntry)
      entry.c0 := c0Signed.S(27.W)
      entry.c1 := c1Signed.S(17.W)
      entry.c2 := c2Signed.S(13.W)
      entry
    }
  }
 
  val lutPath = sys.env.getOrElse("LUT_PATH", "./generators/saturn/src/main/scala/exu/fp/lut")
  println(System.getProperty("user.dir"))

  val exp2LUT      = VecInit(loadLUT(s"$lutPath/exp2-coeffs.txt",       Function.EXP2.c0Sign,  Function.EXP2.c1Sign,  Function.EXP2.c2Sign))
  val sinLUT       = VecInit(loadLUT(s"$lutPath/sin-coeffs.txt",        Function.SIN.c0Sign,   Function.SIN.c1Sign,   Function.SIN.c2Sign))

  val op    = io.in.bits.op
  val index = io.in.bits.index
 
  val entry = MuxLookup(op, exp2LUT(index(5, 0)))(Seq(
    SFUOp.EXP2  -> exp2LUT(index(5, 0)),
    SFUOp.SIN   -> sinLUT(index(5, 0)),
    SFUOp.COS   -> sinLUT(index(5, 0))
  ))
 
  val s1 = Wire(Decoupled(new LookupToPoly))
  val s1Pipe = s1.handshakePipeIf(true)
 
  io.in.ready        := s1.ready
  s1.valid           := io.in.valid
  s1.bits.c0         := entry.c0
  s1.bits.c1         := entry.c1
  s1.bits.c2         := entry.c2
  s1.bits.op         := io.in.bits.op
  s1.bits.xl         := io.in.bits.xl
  s1.bits.exp        := io.in.bits.exp
  s1.bits.sign       := io.in.bits.sign
  s1.bits.bypass     := io.in.bits.bypass
  s1.bits.bypassVal  := io.in.bits.bypassVal
 
  io.out <> s1Pipe
}

class Poly extends Module {
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new LookupToPoly))
    val out = Decoupled(new PolyToCompose)
  })
 
  // Stage 1: Compute and truncate xl^2
  val xl2      = io.in.bits.xl * io.in.bits.xl
  val shift0   = Function.getShift0(io.in.bits.op)
  val aligned0 = (xl2 >> shift0)(14, 0)
 
  val s1 = Wire(Decoupled(new Bundle {
    val op        = UInt(3.W)
    val c0        = SInt(27.W)
    val c1        = SInt(17.W)
    val c2        = SInt(13.W)
    val xl        = UInt(17.W)
    val xl2       = UInt(15.W)
    val exp       = SInt(8.W)
    val sign      = UInt(1.W)
    val bypass    = Bool()
    val bypassVal = UInt(32.W)
  }))
  val s1Pipe = s1.handshakePipeIf(true)
 
  io.in.ready        := s1.ready
  s1.valid           := io.in.valid
  s1.bits.c0         := io.in.bits.c0
  s1.bits.c1         := io.in.bits.c1
  s1.bits.c2         := io.in.bits.c2
  s1.bits.xl         := io.in.bits.xl
  s1.bits.xl2        := aligned0
  s1.bits.op         := io.in.bits.op
  s1.bits.exp        := io.in.bits.exp
  s1.bits.sign       := io.in.bits.sign
  s1.bits.bypass     := io.in.bits.bypass
  s1.bits.bypassVal  := io.in.bits.bypassVal
 
  // Stage 2: Multiply
  val xlSigned  = Cat(0.U(1.W), s1Pipe.bits.xl).asSInt
  val xl2Signed = Cat(0.U(1.W), s1Pipe.bits.xl2).asSInt

  val c2Xl2 = s1Pipe.bits.c2 * xl2Signed
  val c1Xl  = s1Pipe.bits.c1 * xlSigned
 
  val s2 = Wire(Decoupled(new Bundle {
    val op        = UInt(3.W)
    val c0        = SInt(27.W)
    val c1Xl      = SInt(35.W)
    val c2Xl2     = SInt(29.W)
    val exp       = SInt(8.W)
    val sign      = UInt(1.W)
    val bypass    = Bool()
    val bypassVal = UInt(32.W)
  }))
  val s2Pipe = s2.handshakePipeIf(true)
 
  s2.valid           := s1Pipe.valid
  s2.bits.op         := s1Pipe.bits.op
  s2.bits.c0         := s1Pipe.bits.c0
  s2.bits.c1Xl       := c1Xl
  s2.bits.c2Xl2      := c2Xl2
  s2.bits.exp        := s1Pipe.bits.exp
  s2.bits.sign       := s1Pipe.bits.sign
  s2.bits.bypass     := s1Pipe.bits.bypass
  s2.bits.bypassVal  := s1Pipe.bits.bypassVal
  s1Pipe.ready       := s2.ready
 
  // Stage 3: Align And Sum
  val shift1 = Function.getShift1(s2Pipe.bits.op)
  val shift2 = Function.getShift2(s2Pipe.bits.op)
 
  val aligned1 = (s2Pipe.bits.c1Xl  >> shift1).asSInt
  val aligned2 = (s2Pipe.bits.c2Xl2 >> shift2).asSInt
 
  val result = (s2Pipe.bits.c0 + aligned1 + aligned2)(26, 0)
 
  val s3     = Wire(Decoupled(new PolyToCompose))
  val s3Pipe = s3.handshakePipeIf(true)
 
  s3.valid           := s2Pipe.valid
  s2Pipe.ready       := s3.ready
  s3.bits.polyResult := result
  s3.bits.op         := s2Pipe.bits.op
  s3.bits.exp        := s2Pipe.bits.exp
  s3.bits.sign       := s2Pipe.bits.sign
  s3.bits.bypass     := s2Pipe.bits.bypass
  s3.bits.bypassVal  := s2Pipe.bits.bypassVal
 
  io.out <> s3Pipe
}

class Compose extends Module {
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new PolyToCompose))
    val out = Decoupled(new SFUOutput)
  })

  val sign       = io.in.bits.sign
  val exp        = io.in.bits.exp
  val polyResult = io.in.bits.polyResult

  // log2 sin ans cos special compose
  val sum     = Cat(exp.asUInt, polyResult(25, 0))
  val sumAbs  = Mux(exp(7), (~sum + 1.U(34.W)), sum)
  val lzd     = PriorityEncoder(Reverse(sumAbs))

  val expSin   = (134.U(8.W) - lzd).asUInt(7, 0)
  val mantSin  = (sumAbs << lzd)(32, 10)

  val expExp2  = (127.S + exp).asUInt(7, 0)
  val mantExp2 = polyResult(24, 2)

  val result = MuxLookup(io.in.bits.op, 0.U(32.W)) (Seq(
    SFUOp.EXP2  -> Cat(0.U(1.W), expExp2, mantExp2),
    SFUOp.SIN   -> Mux(polyResult(26), Cat(sign, 127.U(8.W) , 0.U(23.W)), Cat(sign, expSin, mantSin)),
    SFUOp.COS   -> Mux(polyResult(26), Cat(sign, 127.U(8.W) , 0.U(23.W)), Cat(sign, expSin, mantSin))
  ))

  val s1     = Wire(Decoupled(new SFUOutput))
  val s1Pipe = s1.handshakePipeIf(true)
  io.in.ready        := s1.ready
  s1.valid           := io.in.valid
  s1.bits.result     := Mux(io.in.bits.bypass, io.in.bits.bypassVal, result)

  io.out <> s1Pipe
}

class SFU extends Module {
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new SFUInput))
    val out = Decoupled(new SFUOutput)
  })
  
  // Stage 0: Filter
  val filter = Module(new Filter)
  io.in.ready        := filter.io.in.ready
  filter.io.in.valid := io.in.valid
  filter.io.in.bits  := io.in.bits
 
  // Stage 1: RangeReduce
  val rangeReduce = Module(new RangeReduce)
  rangeReduce.io.in <> filter.io.out
 
  // Stage 2: LUT
  val lut = Module(new LookupTable)
  lut.io.in <> rangeReduce.io.out
 
  // Stage 3-5: Poly (3 cycles)
  val poly = Module(new Poly)
  poly.io.in <> lut.io.out
 
  // Stage 6: Compose
  val compose = Module(new Compose)
  compose.io.in <> poly.io.out

  compose.io.out.ready := io.out.ready
  io.out.valid         := compose.io.out.valid
  io.out.bits          := compose.io.out.bits
}

// object SFUGen extends App {
//   ChiselStage.emitSystemVerilogFile(
//     new SFU,
//     Array("--target-dir", "generated"),
//     Array("-lowering-options=disallowLocalVariables")
//   )
// }
