
//---------------------- Transaction Class --------------------------
class transaction;
  rand bit oper;
  bit wr;
  bit rd;
  bit [7:0] din;
  bit [7:0] dout;
  bit full;
  bit empty;

  constraint oper_ctrl {
    oper dist {1 := 50, 0 := 50};
  }
endclass

//---------------------- Generator Class ----------------------------
class generator;
  transaction tr;
  mailbox #(transaction) mbx;
  int count = 0;
  int i = 0;
  event next;
  event done;

  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
    tr = new();
  endfunction

  task run();
    repeat (count) begin
      assert(tr.randomize()) else $error("Randomization failed");
      i++;
      mbx.put(tr);
      $display("[GEN]: Operation = %0d, Iteration = %0d", tr.oper, i);
      @(next);
    end
    ->done;
  endtask
endclass

//---------------------- Driver Class -------------------------------
class driver;
  transaction datac;
  virtual fifo_if fif;
  mailbox #(transaction) mbx;

  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
  endfunction

  task reset();
    repeat (1) @(posedge fif.clk);
    fif.rst <= 1;
    @(posedge fif.clk);
    fif.rst <= 1;
    fif.wr <= 0;
    fif.rd <= 0;
    fif.din <= 0;
    repeat (3) @(posedge fif.clk);
    fif.rst <= 0;
    $display("[DRV]: DUT Reset Done");
  endtask

  task write();
    @(posedge fif.clk);
    fif.wr <= 1;
    fif.rd <= 0;
    fif.rst <= 0;
    fif.din <= $urandom_range(1, 40);
    @(posedge fif.clk);
    fif.wr <= 0;
    $display("[DRV]: Data Written = %0d", fif.din);
    @(posedge fif.clk);
  endtask

  task read();
    @(posedge fif.clk);
    fif.wr <= 0;
    fif.rd <= 1;
    fif.rst <= 0;
    @(posedge fif.clk);
    fif.rd <= 0;
    $display("[DRV]: Read Operation Triggered");
    @(posedge fif.clk);
  endtask

  task run();
    forever begin
      mbx.get(datac);
      if (datac.oper == 1)
        write();
      else
        read();
    end
  endtask
endclass

//---------------------- Monitor Class ------------------------------
class monitor;
  transaction tr;
  mailbox #(transaction) mbx;
  virtual fifo_if fif;

  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
  endfunction

  task run();
    tr = new();
    forever begin
      repeat(1) @(posedge fif.clk);
      tr.wr = fif.wr;
      tr.rd = fif.rd;
      tr.din = fif.din;
      tr.full = fif.full;
      tr.empty = fif.empty;
      @(posedge fif.clk);
      tr.dout = fif.dout;
      mbx.put(tr);
      $display("[MON]: RD=%0d, WR=%0d, DIN=%0d, DOUT=%0d, FULL=%0d, EMPTY=%0d", tr.rd, tr.wr, tr.din, tr.dout, tr.full, tr.empty);
    end
  endtask
endclass

//---------------------- Scoreboard Class ---------------------------
class scoreboard;
  transaction tr;
  mailbox #(transaction) mbx;
  event next;
  bit [7:0] temp;
  bit [7:0] din[$];
  int er = 0;

  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
  endfunction

  task run();
    forever begin
      mbx.get(tr);
      $display("[SCO]: RD=%0d, WR=%0d, DIN=%0d, DOUT=%0d, FULL=%0d, EMPTY=%0d", tr.rd, tr.wr, tr.din, tr.dout, tr.full, tr.empty);

      if (tr.wr == 1) begin
        if (!tr.full) begin
          din.push_front(tr.din);
          $display("[SCO]: Data Stored in Queue: %0d", tr.din);
        end else begin
          $display("[SCO]: FIFO Full - Write Ignored");
        end
      end

      if (tr.rd == 1) begin
        if (!tr.empty) begin
          temp = din.pop_back();
          if (temp == tr.dout) begin
            $display("[SCO]: Data Matched: %0d", temp);
          end else begin
            $error("[SCO]: Data Mismatch! Expected = %0d, Got = %0d", temp, tr.dout);
            er++;
          end
        end else begin
          $display("[SCO]: FIFO Empty - Read Ignored");
        end
      end

      ->next;
    end
  endtask
endclass

//---------------------- Environment Class --------------------------
class environment;
  generator gen;
  driver dr;
  scoreboard sc;
  monitor mon;
  mailbox #(transaction) gdmbx;
  mailbox #(transaction) msmbx;
  event gentoscore;
  virtual fifo_if fif;

  function new(virtual fifo_if fif);
    this.fif = fif;
    gdmbx = new();
    msmbx = new();
    gen = new(gdmbx);
    dr = new(gdmbx);
    mon = new(msmbx);
    sc = new(msmbx);
    dr.fif = fif;
    mon.fif = fif;
    gen.next = gentoscore;
    sc.next = gentoscore;
  endfunction

  task pre_test();
    dr.reset();
  endtask

  task test();
    fork
      mon.run();
      gen.run();
      sc.run();
      dr.run();
    join_any
  endtask

  task post_test();
    wait(gen.done.triggered);
    $display("==========================================");
    $display("Simulation Complete. Errors Found: %0d", sc.er);
    $display("==========================================");
    $finish();
  endtask

  task run();
    pre_test();
    test();
    post_test();
  endtask
endclass

//---------------------- Interface --------------------------
interface fifo_if();
  logic clk;
  logic rst;
  logic wr;
  logic rd;
  logic [7:0] din;
  logic [7:0] dout;
  logic empty;
  logic full;
endinterface

//---------------------- DUT Module (Placeholder) --------------------------


module testbench;
  fifo_if fif();
  environment env;

  FIFO dut (
    .clk(fif.clk),
    .rst(fif.rst),
    .wr(fif.wr),
    .rd(fif.rd),
    .din(fif.din),
    .dout(fif.dout),
    .empty(fif.empty),
    .full(fif.full)
  );

  initial begin
    fif.clk = 0;
    forever #50 fif.clk = ~fif.clk;  // 20ns clock period
  end

  initial begin
    env = new(fif);
    env.gen.count = 30;
    env.run();
  end

  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
  end
endmodule
