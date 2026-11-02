package saturn.mem

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.diplomacy._

import saturn.common._
import cute.trace.{CUTETraceContext, CUTETraceParams}
import cute.trace.generated.CUTETrace

class TLInterface(tagBits: Int, tracePort: Int = 0)(implicit p: Parameters) extends LazyModule()(p) with HasCoreParameters {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
    name      = s"Core ${tileId} Vector Load",
    sourceId  = IdRange(0, 1 << tagBits)
  )))))
  override lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {

    val (out, edge) = node.out(0)

    val widthBytes = edge.slave.beatBytes
    val offBits = log2Ceil(widthBytes)
    private val traceCycle = RegInit(0.U(64.W))
    traceCycle := traceCycle + 1.U
    private val systemTrace = PlusArg("cute_sys_trace", default = 0).orR
    private val traceEventSeq = RegInit(0.U(64.W))
    private val traceNextSeq = traceEventSeq + out.a.fire.asUInt + out.d.fire.asUInt
    traceEventSeq := traceNextSeq
    private implicit val traceContext: CUTETraceContext =
      CUTETraceContext(traceCycle, CUTETraceParams(enable = true))
    CUTETrace.SystemMemory.request(
      cond = systemTrace && out.a.fire,
      hart = tileId.U, engine = 1.U, port = tracePort.U,
      event_seq = traceEventSeq,
      source = out.a.bits.source, opcode = out.a.bits.opcode,
      address = out.a.bits.address, lg_size = out.a.bits.size,
      width_bytes = widthBytes.U, mask = out.a.bits.mask, data = out.a.bits.data)
    CUTETrace.SystemMemory.response(
      cond = systemTrace && out.d.fire,
      hart = tileId.U, engine = 1.U, port = tracePort.U,
      event_seq = traceEventSeq + out.a.fire.asUInt,
      source = out.d.bits.source, opcode = out.d.bits.opcode,
      lg_size = out.d.bits.size, width_bytes = widthBytes.U,
      denied = out.d.bits.denied.asUInt, corrupt = out.d.bits.corrupt.asUInt,
      data = out.d.bits.data)

    val io = IO(new Bundle {
      val busy = Output(Bool())
      val req = Flipped(Decoupled(new MemRequest(widthBytes, tagBits)))
      val resp = Valid(new MemResponse(widthBytes, tagBits))
    })

    // All source IDs may be occupied: represent 2^tagBits, not just 2^tagBits-1.
    val inflights = RegInit(0.U((tagBits + 1).W))
    CUTETrace.SystemMemory.progress(
      cond = systemTrace && traceCycle(7, 0) === 0.U,
      hart = tileId.U, engine = 1.U, port = tracePort.U,
      next_seq = traceNextSeq,
      outstanding = inflights + out.a.fire.asUInt - out.d.fire.asUInt)
    when (out.a.fire || out.d.fire) {
      inflights := inflights + out.a.fire - out.d.fire
    }
    io.busy := inflights =/= 0.U

    io.req.ready := out.a.ready
    out.a.valid := io.req.valid
    out.a.bits := Mux(io.req.bits.store,
      edge.Put(
        io.req.bits.tag,
        (io.req.bits.addr >> offBits) << offBits,
        log2Ceil(widthBytes).U,
        io.req.bits.data,
        io.req.bits.mask)._2,
      edge.Get(
        io.req.bits.tag,
        (io.req.bits.addr >> offBits) << offBits,
        log2Ceil(widthBytes).U)._2
    )

    val ackDelay = PlusArg("cute_store_ack_delay", default = 0)
    val ackAge = RegInit(0.U(32.W))
    val ackAllowed = out.d.bits.opcode =/= TLMessages.AccessAck || ackAge >= ackDelay
    when(!out.d.valid || out.d.fire) { ackAge := 0.U }
      .elsewhen(!ackAllowed) { ackAge := ackAge + 1.U }
    out.d.ready := ackAllowed
    io.resp.valid := out.d.valid && ackAllowed
    io.resp.bits.data := out.d.bits.data
    io.resp.bits.tag := out.d.bits.source
  }
}


class TLSplitInterface(implicit p: Parameters) extends LazyModule()(p) with HasCoreParameters with HasVectorParams {

  val reader = LazyModule(new TLInterface(dmemTagBits, tracePort = 0))
  val writer = LazyModule(new TLInterface(dmemTagBits, tracePort = 1))

  val arb = LazyModule(new TLXbar)
  def node = TLWidthWidget(mLenB) := arb.node
  def edge = arb.node.edges.out(0)

  arb.node := reader.node
  arb.node := writer.node

  override lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val vec = Flipped(new VectorMemIO)
      val mem_busy = Output(Bool())
    })

    reader.module.io.req <> io.vec.load_req
    io.vec.load_resp <> reader.module.io.resp
    writer.module.io.req <> io.vec.store_req
    io.vec.store_ack <> writer.module.io.resp
    io.mem_busy := reader.module.io.busy || writer.module.io.busy
  }
}
