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
import fudian.{FCMA_ADD_s1, FCMA_ADD_s2, FMUL_s1, FMUL_s2, FMUL_s3, FMULToFADD, RawFloat}
import fudian.utils.Multiplier

object EXP2FP32Parameters {
  val C0        = "h3F800000".U(32.W)
  val C1        = "h3F317218".U(32.W)
  val C2        = "h3E75FDF0".U(32.W)
  val MIN_INPUT = "hC2AE999A".U(32.W)
  val MAX_INPUT = "h42B16666".U(32.W)
  val ZERO      = 0.U(32.W)
  val INF       = "h7F800000".U(32.W)
  val NAN       = "h7FC00000".U(32.W)
}

object EXP2FP32Utils {
  implicit class DecoupledPipe[T <: Data](val decoupledBundle: DecoupledIO[T]) extends AnyVal {
    def handshakePipeIf(en: Boolean): DecoupledIO[T] = {
      if (en) {
        val out = Wire(Decoupled(chiselTypeOf(decoupledBundle.bits)))
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

import EXP2FP32Utils._

class MULFP32[T <: Bundle](ctrlSignals: T) extends Module {
  val expWidth  = 8
  val precision = 24
  class InBundle extends Bundle {
    val a    = UInt(32.W)
    val b    = UInt(32.W)
    val rm   = UInt(3.W)
    val ctrl = ctrlSignals.cloneType.asInstanceOf[T]
  }
  class OutBundle extends Bundle {
    val result = UInt(32.W)
    val toAdd  = new FMULToFADD(expWidth, precision)
    val ctrl   = ctrlSignals.cloneType.asInstanceOf[T]
  }
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new InBundle))
    val out = Decoupled(new OutBundle)
  })
  
  val mul   = Module(new Multiplier(precision + 1, pipeAt = Seq()))
  val mulS1 = Module(new FMUL_s1(expWidth, precision))
  val mulS2 = Module(new FMUL_s2(expWidth, precision))
  val mulS3 = Module(new FMUL_s3(expWidth, precision))
  
  mulS1.io.a  := io.in.bits.a
  mulS1.io.b  := io.in.bits.b
  mulS1.io.rm := io.in.bits.rm
  
  val rawA = RawFloat.fromUInt(io.in.bits.a, expWidth, precision)
  val rawB = RawFloat.fromUInt(io.in.bits.b, expWidth, precision)
  mul.io.a := rawA.sig
  mul.io.b := rawB.sig
  mul.io.regEnables.foreach(_ := true.B)
  
  val s1 = Wire(Decoupled(new Bundle {
    val mulS1Out = mulS1.io.out.cloneType
    val prod     = mul.io.result.cloneType
    val ctrl     = ctrlSignals.cloneType.asInstanceOf[T]
  }))
  val s1Pipe = s1.handshakePipeIf(true)
  s1.valid         := io.in.valid
  s1.bits.mulS1Out := mulS1.io.out
  s1.bits.prod     := mul.io.result
  s1.bits.ctrl     := io.in.bits.ctrl
  io.in.ready      := s1.ready
  
  mulS2.io.in   := s1Pipe.bits.mulS1Out
  mulS2.io.prod := s1Pipe.bits.prod
  
  val s2 = Wire(Decoupled(new Bundle {
    val mulS2Out = mulS2.io.out.cloneType
    val ctrl     = ctrlSignals.cloneType.asInstanceOf[T]
  }))
  val s2Pipe = s2.handshakePipeIf(true)
  s2.valid         := s1Pipe.valid
  s2.bits.mulS2Out := mulS2.io.out
  s2.bits.ctrl     := s1Pipe.bits.ctrl
  s1Pipe.ready     := s2.ready
  
  mulS3.io.in := s2Pipe.bits.mulS2Out
  
  val s3     = Wire(Decoupled(new OutBundle))
  val s3Pipe = s3.handshakePipeIf(true)
  s3.valid          := s2Pipe.valid
  s3.bits.result    := mulS3.io.result
  s3.bits.toAdd     := mulS3.io.to_fadd
  s3.bits.ctrl      := s2Pipe.bits.ctrl
  s2Pipe.ready      := s3.ready
  
  io.out <> s3Pipe
}

class CMAFP32[T <: Bundle](ctrlSignals: T) extends Module {
  val expWidth  = 8
  val precision = 24
  class InBundle extends Bundle {
    val a     = UInt(32.W)
    val b     = UInt(32.W)
    val c     = UInt(32.W)
    val rm    = UInt(3.W)
    val ctrl  = ctrlSignals.cloneType.asInstanceOf[T]
  }
  class OutBundle extends Bundle {
    val result = UInt(32.W)
    val ctrl   = ctrlSignals.cloneType.asInstanceOf[T]
  }
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new InBundle))
    val out = Decoupled(new OutBundle)
  })
  
  class MULToADD extends Bundle {
    val c       = UInt(32.W)
    val topCtrl = ctrlSignals.cloneType.asInstanceOf[T]
  }
  val mul   = Module(new MULFP32[MULToADD](new MULToADD))
  val addS1 = Module(new FCMA_ADD_s1(expWidth, precision * 2, precision))
  val addS2 = Module(new FCMA_ADD_s2(expWidth, precision * 2, precision))
  
  mul.io.in.valid             := io.in.valid
  mul.io.in.bits.a            := io.in.bits.a
  mul.io.in.bits.b            := io.in.bits.b
  mul.io.in.bits.rm           := io.in.bits.rm
  mul.io.in.bits.ctrl.c       := io.in.bits.c
  mul.io.in.bits.ctrl.topCtrl := io.in.bits.ctrl
  io.in.ready                 := mul.io.in.ready
  
  addS1.io.a             := Cat(mul.io.out.bits.ctrl.c, 0.U(precision.W))
  addS1.io.b             := mul.io.out.bits.toAdd.fp_prod.asUInt
  addS1.io.b_inter_valid := true.B
  addS1.io.b_inter_flags := mul.io.out.bits.toAdd.inter_flags
  addS1.io.rm            := mul.io.out.bits.toAdd.rm
  
  val s4 = Wire(Decoupled(new Bundle {
    val out  = addS1.io.out.cloneType
    val ctrl = ctrlSignals.cloneType.asInstanceOf[T]
  }))
  val s4Pipe = s4.handshakePipeIf(true)
  s4.valid         := mul.io.out.valid
  s4.bits.out      := addS1.io.out
  s4.bits.ctrl     := mul.io.out.bits.ctrl.topCtrl
  mul.io.out.ready := s4.ready
  
  addS2.io.in := s4Pipe.bits.out
  
  val s5     = Wire(Decoupled(new OutBundle))
  val s5Pipe = s5.handshakePipeIf(true)
  s5.valid       := s4Pipe.valid
  s5.bits.result := addS2.io.result
  s5.bits.ctrl   := s4Pipe.bits.ctrl
  s4Pipe.ready   := s5.ready
  
  io.out <> s5Pipe
}

class CMAFP32LUTParallel[T <: Bundle](ctrlSignals: T) extends Module {
  val expWidth  = 8
  val precision = 24
  class InBundle extends Bundle {
    val a     = UInt(32.W)
    val b     = UInt(32.W)
    val c     = UInt(32.W)
    val index = UInt(7.W)  // 改为7位（只需要128项）
    val rm    = UInt(3.W)
    val ctrl  = ctrlSignals.cloneType.asInstanceOf[T]
  }
  class OutBundle extends Bundle {
    val result = UInt(32.W)
    val value  = UInt(32.W)
    val ctrl   = ctrlSignals.cloneType.asInstanceOf[T]
  }
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new InBundle))
    val out = Decoupled(new OutBundle)
  })
  
  class MULToADD extends Bundle {
    val c       = UInt(32.W)
    val index   = UInt(7.W)  // 改为7位
    val topCtrl = ctrlSignals.cloneType.asInstanceOf[T]
  }
  val mul   = Module(new MULFP32[MULToADD](new MULToADD))
  val addS1 = Module(new FCMA_ADD_s1(expWidth, precision * 2, precision))
  val addS2 = Module(new FCMA_ADD_s2(expWidth, precision * 2, precision))
  
  mul.io.in.valid             := io.in.valid
  mul.io.in.bits.a            := io.in.bits.a
  mul.io.in.bits.b            := io.in.bits.b
  mul.io.in.bits.rm           := io.in.bits.rm
  mul.io.in.bits.ctrl.c       := io.in.bits.c
  mul.io.in.bits.ctrl.index   := io.in.bits.index
  mul.io.in.bits.ctrl.topCtrl := io.in.bits.ctrl
  io.in.ready                 := mul.io.in.ready
  
  addS1.io.a             := Cat(mul.io.out.bits.ctrl.c, 0.U(precision.W))
  addS1.io.b             := mul.io.out.bits.toAdd.fp_prod.asUInt
  addS1.io.b_inter_valid := true.B
  addS1.io.b_inter_flags := mul.io.out.bits.toAdd.inter_flags
  addS1.io.rm            := mul.io.out.bits.toAdd.rm
  
  val s4 = Wire(Decoupled(new Bundle {
    val out   = addS1.io.out.cloneType
    val index = UInt(7.W)  // 改为7位
    val ctrl  = ctrlSignals.cloneType.asInstanceOf[T]
  }))
  val s4Pipe = s4.handshakePipeIf(true)
  s4.valid         := mul.io.out.valid
  s4.bits.out      := addS1.io.out
  s4.bits.index    := mul.io.out.bits.ctrl.index
  s4.bits.ctrl     := mul.io.out.bits.ctrl.topCtrl
  mul.io.out.ready := s4.ready
  
  addS2.io.in := s4Pipe.bits.out
  
  val s5     = Wire(Decoupled(new OutBundle))
  val s5Pipe = s5.handshakePipeIf(true)
  s5.valid       := s4Pipe.valid
  s5.bits.result := addS2.io.result
  s5.bits.ctrl   := s4Pipe.bits.ctrl
  s4Pipe.ready   := s5.ready
  
  // LUT 改为只存储 [0, 1) 范围，128项
  val table = VecInit((0 until 128).map { i =>
    val x = i.toDouble / 128.0  // x ∈ [0, 1)
    val y = Math.pow(2.0, x)
    val bits = java.lang.Float.floatToIntBits(y.toFloat)
    bits.U(32.W)
  })
  
  s5.bits.value := table(s4Pipe.bits.index)
  io.out <> s5Pipe
}

class DecomposeFP32FloorNeg[T <: Bundle](ctrlSignals: T) extends Module {
  class InBundle extends Bundle {
    val y    = UInt(32.W)
    val ctrl = ctrlSignals.cloneType.asInstanceOf[T]
  }
  class OutBundle extends Bundle {
    val yi   = SInt(9.W)
    val yfi  = UInt(7.W)
    val yfj  = UInt(32.W)
    val ctrl = ctrlSignals.cloneType.asInstanceOf[T]
  }
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new InBundle))
    val out = Decoupled(new OutBundle)
  })
  
  val expWidth  = 8
  val precision = 24
  val raw = RawFloat.fromUInt(io.in.bits.y, expWidth, precision)
  val sign = raw.sign
  val exp  = raw.exp
  val sig  = raw.sig
  
  val expSigned = (exp.zext - 127.S).asSInt
  val section1 = expSigned <= -8.S
  
  // ===========================
  // Section 1: expSigned <= -8
  // ===========================
  val yiS1  = 0.S(9.W)
  val yfiS1 = 0.U(7.W)
  val yfjS1 = io.in.bits.y
  
  // ===========================
  // Section 2: expSigned ∈ [-7, 7]
  // ===========================
  val sigExtended = Cat(0.U(7.W), sig, 0.U(7.W))  // 扩展足够位宽
  val sigShifted = Mux(expSigned >= 0.S,
    sigExtended << expSigned.asUInt,
    sigExtended >> (-expSigned).asUInt
  )
  
  // 提取整数和小数部分（30位小数足够表示）
  val intPart = sigShifted(37, 30)  // 8位整数
  val fracPart = sigShifted(29, 0) // 30位小数（包含 fracHigh 7位 + fracLow 23位）
  
  // 检查小数部分是否非零
  val hasFrac = fracPart =/= 0.U
  
  // 对于负数且有小数：
  // - 整数部分：intPart + 1
  // - 小数部分：(1 << 30) - fracPart
  val intPartFinal = Mux(sign && hasFrac, intPart + 1.U, intPart)
  val fracPartFinal = Mux(sign && hasFrac, (1.U << 30) - fracPart, fracPart)
  
  // 从 fracPartFinal 提取 fracHigh (高7位) 和 fracLow (低23位)
  val fracHigh = fracPartFinal(29, 23)  // 高7位
  val fracLow = fracPartFinal(22, 0)    // 低23位
  
  // 归一化 fracLow 成 FP32
  val fracLowIsZero = fracLow === 0.U
  val lzdCount = PriorityEncoder(Reverse(fracLow))
  val expBiased = Mux(fracLowIsZero, 0.U(8.W), (119.U(8.W) - lzdCount)(7, 0))
  val mantissa = Mux(fracLowIsZero, 0.U(23.W), (fracLow << (lzdCount + 1.U))(22, 0))
  
  val yiS2  = Mux(sign, -intPartFinal.zext.asSInt, intPartFinal.zext.asSInt)
  val yfiS2 = fracHigh
  val yfjS2 = Cat(0.U(1.W), expBiased, mantissa)
  
  val s1 = Wire(Decoupled(new OutBundle))
  val s1Pipe = s1.handshakePipeIf(true)
  
  s1.valid     := io.in.valid
  s1.bits.yi   := Mux(section1, yiS1, yiS2)
  s1.bits.yfi  := Mux(section1, yfiS1, yfiS2)
  s1.bits.yfj  := Mux(section1, yfjS1, yfjS2)
  s1.bits.ctrl := io.in.bits.ctrl
  io.in.ready  := s1.ready
  
  io.out <> s1Pipe
}

class FilterFP32[T <: Bundle](ctrlSignals: Bundle) extends Module {
  class InBundle extends Bundle {
    val in = UInt(32.W)
    val ctrl = ctrlSignals.cloneType.asInstanceOf[T]
  }
  class OutBundle extends Bundle {
    val out       = UInt(32.W)
    val bypass    = Bool()
    val bypassVal = UInt(32.W)
    val ctrl      = ctrlSignals.cloneType.asInstanceOf[T]
  }
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new InBundle))
    val out = Decoupled(new OutBundle)
  })
  
  val s = io.in.bits.in(31)
  val e = io.in.bits.in(30, 23)
  val f = io.in.bits.in(22, 0)
  
  val isInfPos = (e === "hFF".U) && (f === 0.U) && (s === 0.U)
  val isInfNeg = (e === "hFF".U) && (f === 0.U) && (s === 1.U)
  val isNaN    = (e === "hFF".U) && (f =/= 0.U)
  
  val tooBig = (!s) && (io.in.bits.in > EXP2FP32Parameters.MAX_INPUT)
  val tooNeg =   s  && (io.in.bits.in > EXP2FP32Parameters.MIN_INPUT)
  
  val bypass = isNaN || isInfPos || isInfNeg || tooBig || tooNeg
  
  val bypassVal = Wire(UInt(32.W))
  when (isNaN) {
    bypassVal := EXP2FP32Parameters.NAN
  }.elsewhen (isInfPos || tooBig) {
    bypassVal := EXP2FP32Parameters.INF
  }.elsewhen (isInfNeg || tooNeg) {
    bypassVal := EXP2FP32Parameters.ZERO
  }.otherwise {
    bypassVal := EXP2FP32Parameters.ZERO
  }
  
  val s1 = Wire(Decoupled(new OutBundle))
  val s1Pipe = s1.handshakePipeIf(true)
  
  s1.valid          := io.in.valid
  s1.bits.out       := io.in.bits.in
  s1.bits.bypass    := bypass
  s1.bits.bypassVal := bypassVal
  s1.bits.ctrl      := io.in.bits.ctrl
  io.in.ready       := s1.ready
  
  io.out <> s1Pipe
}

class EXP2FP32 extends Module {
  class InBundle extends Bundle {
    val in = UInt(32.W)
    val rm = UInt(3.W)
  }
  class OutBundle extends Bundle {
    val out = UInt(32.W)
  }
  val io = IO(new Bundle {
    val in  = Flipped(Decoupled(new InBundle))
    val out = Decoupled(new OutBundle)
  })
  
  class FilterToDecompose extends Bundle {
    val rm = UInt(3.W)
  }
  
  val filter = Module(new FilterFP32[FilterToDecompose](new FilterToDecompose))
  io.in.ready               := filter.io.in.ready
  filter.io.in.valid        := io.in.valid
  filter.io.in.bits.in      := io.in.bits.in
  filter.io.in.bits.ctrl.rm := io.in.bits.rm
  
  class DecomposeToCMA0 extends Bundle {
    val rm        = UInt(3.W)
    val bypass    = Bool()
    val bypassVal = UInt(32.W)
  }
  
  val decompose = Module(new DecomposeFP32FloorNeg[DecomposeToCMA0](new DecomposeToCMA0))
  filter.io.out.ready                 := decompose.io.in.ready
  decompose.io.in.valid               := filter.io.out.valid
  decompose.io.in.bits.y              := filter.io.out.bits.out
  decompose.io.in.bits.ctrl.rm        := filter.io.out.bits.ctrl.rm
  decompose.io.in.bits.ctrl.bypass    := filter.io.out.bits.bypass
  decompose.io.in.bits.ctrl.bypassVal := filter.io.out.bits.bypassVal
  
  class CMA0ToCMA1 extends Bundle {
    val rm        = UInt(3.W)
    val bypass    = Bool()
    val bypassVal = UInt(32.W)
    val yi        = SInt(9.W)
    val yfi       = UInt(7.W)
    val yfj       = UInt(32.W)
  }
  
  val cma0 = Module(new CMAFP32[CMA0ToCMA1](new CMA0ToCMA1))
  decompose.io.out.ready         := cma0.io.in.ready
  cma0.io.in.valid               := decompose.io.out.valid
  cma0.io.in.bits.a              := decompose.io.out.bits.yfj
  cma0.io.in.bits.b              := EXP2FP32Parameters.C2
  cma0.io.in.bits.c              := EXP2FP32Parameters.C1
  cma0.io.in.bits.rm             := decompose.io.out.bits.ctrl.rm
  cma0.io.in.bits.ctrl.rm        := decompose.io.out.bits.ctrl.rm
  cma0.io.in.bits.ctrl.bypass    := decompose.io.out.bits.ctrl.bypass
  cma0.io.in.bits.ctrl.bypassVal := decompose.io.out.bits.ctrl.bypassVal
  cma0.io.in.bits.ctrl.yi        := decompose.io.out.bits.yi
  cma0.io.in.bits.ctrl.yfi       := decompose.io.out.bits.yfi
  cma0.io.in.bits.ctrl.yfj       := decompose.io.out.bits.yfj
  
  class CMA1ToMUL1 extends Bundle {
    val rm        = UInt(3.W)
    val bypass    = Bool()
    val bypassVal = UInt(32.W)
    val yi        = SInt(9.W)
  }
  
  val cma1 = Module(new CMAFP32LUTParallel[CMA1ToMUL1](new CMA1ToMUL1))
  cma0.io.out.ready              := cma1.io.in.ready
  cma1.io.in.valid               := cma0.io.out.valid
  cma1.io.in.bits.a              := cma0.io.out.bits.ctrl.yfj
  cma1.io.in.bits.b              := cma0.io.out.bits.result
  cma1.io.in.bits.c              := EXP2FP32Parameters.C0
  cma1.io.in.bits.index          := cma0.io.out.bits.ctrl.yfi
  cma1.io.in.bits.rm             := cma0.io.out.bits.ctrl.rm
  cma1.io.in.bits.ctrl.rm        := cma0.io.out.bits.ctrl.rm
  cma1.io.in.bits.ctrl.bypass    := cma0.io.out.bits.ctrl.bypass
  cma1.io.in.bits.ctrl.bypassVal := cma0.io.out.bits.ctrl.bypassVal
  cma1.io.in.bits.ctrl.yi        := cma0.io.out.bits.ctrl.yi
  
  class MUL1ToMux extends Bundle {
    val bypass    = Bool()
    val bypassVal = UInt(32.W)
    val yi        = SInt(9.W)
  }
  
  val mul1 = Module(new MULFP32[MUL1ToMux](new MUL1ToMux))
  cma1.io.out.ready              := mul1.io.in.ready
  mul1.io.in.valid               := cma1.io.out.valid
  mul1.io.in.bits.a              := cma1.io.out.bits.result
  mul1.io.in.bits.b              := cma1.io.out.bits.value
  mul1.io.in.bits.rm             := cma1.io.out.bits.ctrl.rm
  mul1.io.in.bits.ctrl.bypass    := cma1.io.out.bits.ctrl.bypass
  mul1.io.in.bits.ctrl.bypassVal := cma1.io.out.bits.ctrl.bypassVal
  mul1.io.in.bits.ctrl.yi        := cma1.io.out.bits.ctrl.yi
  
  val fResult   = mul1.io.out.bits.result
  val yi        = mul1.io.out.bits.ctrl.yi
  val bypass    = mul1.io.out.bits.ctrl.bypass
  val bypassVal = mul1.io.out.bits.ctrl.bypassVal
  
  val ef   = fResult(30, 23)
  val mant = fResult(22, 0)
  val e    = (ef.zext.asSInt + yi)(7, 0).asUInt
  
  val mainOut = Cat(0.U(1.W), e, mant)
  val out     = Mux(bypass, bypassVal, mainOut)
  
  val s18 = Wire(Decoupled(new OutBundle))
  val s18Pipe = s18.handshakePipeIf(true)
  
  s18.valid         := mul1.io.out.valid
  s18.bits.out      := out
  mul1.io.out.ready := s18.ready
  
  io.out <> s18Pipe
}

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
  ).map(_.restrictSEW(1,2,3)).flatten.map(_.pipelined(17))

  def generate(implicit p: Parameters) = new FPSFuncPipe()(p)
}

class FPSFuncPipe(implicit p : Parameters) extends PipelinedFunctionalUnit(17)(p) with HasFPUParameters {
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

  // val outexc = Wire(Vec(nTandemFMA, UInt(5.W)))
  val out = Wire(Vec(nTandemFMA, UInt(32.W)))
  val exp2outValid = Wire(Vec(nTandemFMA, Bool()))

  val pipe_out = (0 until nTandemFMA).map {i =>

    val exp2_opensource = Module(new EXP2FP32)

    // val exp2 = Module(new VEXP2FAKE)

    val rvs2_bits = vec_rvs2(i)

    val iss_fire_pipe = Reg(Bool())
    iss_fire_pipe := io.iss.valid

	exp2_opensource.io.in.valid := iss_fire_pipe && exp2_op
    exp2_opensource.io.in.bits.in := Mux(valid && exp2_op, vec_rvs2(i), 0.U)
    exp2_opensource.io.in.bits.rm := op.frm
    exp2_opensource.io.out.ready := true.B

    // exp2.io.in_valid := iss_fire_pipe && exp2_op
    // exp2.io.rvs2_input := Mux(valid && exp2_op, rvs2_bits, 0.U)

    val exp2_out_valid = exp2_opensource.io.out.valid
    val exp2_out = exp2_opensource.io.out.bits.out

    // val elemout = Mux1H(
    //   Seq(vfclass_inst, vfrsqrt7_inst, vfrec7_inst, divsqrt_valid),
    //   Seq(gen_vfclass, recSqrt7.io.out, rec7.io.out, divsqrt_reg)
    // )(63,0)

    // outexc(i) := Mux(exp2_out_valid, exp2.io.exc,
    //       0.U)
    exp2outValid(i) := exp2_out_valid
    out(i) := exp2_out
  }

  io.set_fflags.valid := exp2outValid.asUInt.andR
  io.set_fflags.bits := 0.U.asTypeOf(io.set_fflags.bits)

  io.scalar_write.valid := false.B
  io.scalar_write.bits := DontCare

  io.write.valid := io.pipe(depth-1).valid
  io.write.bits.eg := io.pipe(depth-1).bits.wvd_eg
  io.write.bits.mask := FillInterleaved(8, io.pipe(depth-1).bits.wmask)
  io.write.bits.data := out.asUInt
  io.stall := false.B
}
