package saturn.shuttle

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.diplomacy._

import saturn.common._
import saturn.backend.{VectorBackend}
import saturn.mem.{TLSplitInterface, SGTLInterface, VectorMemUnit}
import saturn.frontend.{VectorDispatcher}
import shuttle.common._


class SaturnShuttleUnit(implicit p: Parameters) extends ShuttleVectorUnit()(p) with HasVectorParams with HasCoreParameters {
  assert(!vParams.useScalarFPFMA)
  if (vParams.useScalarFPFMA) {
    require(coreParams.fpu.get.dfmaLatency == vParams.fmaPipeDepth - 1)
  }

  val tl_if = LazyModule(new TLSplitInterface)
  atlNode := TLBuffer(vParams.tlBuffer) := TLWidthWidget(mLenB) := tl_if.node

  val sg_if = sgNode.map { n =>
    val sg_if = LazyModule(new SGTLInterface)
    n :=* sg_if.node
    sg_if
  }

  override lazy val module = new SaturnShuttleImpl
  class SaturnShuttleImpl extends ShuttleVectorUnitModuleImp(this) with HasVectorParams with HasCoreParameters {

    val dis = Module(new VectorDispatcher)
    val scalar_arb = Module(new Arbiter(new ScalarWrite, 2))
    val vfu = Module(new SaturnShuttleFrontend(sgSize, tl_if.edge))
    val vu = Module(new VectorBackend)
    val vmu = Module(new VectorMemUnit(sgSize))

    sg_if.foreach { sg =>
      sg.module.io.vec <> vmu.io.sgmem.get
    }

    dis.io.issue <> vfu.io.issue //所有issue的指令都在这里，当某条访存指令的最后一个mop发射，这条指令也完成了退休，和其他的rocc等指令也是同步抵达张量模块。
    vfu.io.core <> io
    vfu.io.sg_base := io_sg_base

    vu.io.index_access <> vfu.io.index_access
    vu.io.mask_access <> vfu.io.mask_access
    vu.io.vmu <> vmu.io.vu //这里把load和store相关的线连到vmu去了
    vu.io.vat_tail := dis.io.vat_tail//新增一条指令，这个就+1，用这个追标记的访存指令
    vu.io.vat_head := dis.io.vat_head//得追这个，这个每次最多+1，每完成一条指令，这个就移，标注好tensor关注的tail，等tail过了，flag就过了。
    vu.io.dis <> dis.io.dis //这个dis.io.dis.ready能够控制前端所有指令进入vu和vmu
    //只需要把dis.io.dis.fire 拿到数据，然后接到外部的总控制单元，每周期根据realease来释放对应的flag指令。
    //另外还需要一组和cutev3进行控制交互的线束组用来控制dis的ready
    dis.io.tensor_block := false.B
    dis.io.vat_release := vu.io.vat_release
    vmu.io.enq <> dis.io.mem

    vmu.io.scalar_check <> vfu.io.scalar_check

    io.backend_busy   := vu.io.busy || tl_if.module.io.mem_busy || sg_if.map(_.module.io.mem_busy).getOrElse(false.B) || vmu.io.busy
    io.set_vxsat      := vu.io.set_vxsat
    io.set_fflags     := vu.io.set_fflags


    scalar_arb.io.in(0) <> vu.io.scalar_resp
    scalar_arb.io.in(1) <> dis.io.scalar_resp
    io.resp <> Queue(scalar_arb.io.out)

    tl_if.module.io.vec <> vmu.io.dmem

    vu.io.fp_req.ready := false.B
    vu.io.fp_resp.valid := false.B
    vu.io.fp_resp.bits := DontCare

    // performance counters
    val time_stamp = RegInit(0.U(40.W))
      time_stamp := time_stamp + 1.U

    if (vParams.enablePerfCounter) {
      printf("[Saturn_VPU_perf %d] %x %x %x  \n", 
        time_stamp,
        vu.io.vxu_busy,
        vmu.io.busy,
        io.backend_busy
      )
    }

  }
}
