// ------------------------------------
// Interface
// ------------------------------------
interface dp_if(input logic clk);
  logic rst;
  logic enable, wr, rd;
  logic [5:0] wr_addr, rd_addr;
  logic [7:0] wr_data;
  logic [7:0] rd_data;
endinterface

// ------------------------------------
// Transaction Class
// ------------------------------------
class dp_transaction;
  rand bit enable, wr, rd;
  rand bit [5:0] wr_addr, rd_addr;
  rand bit [7:0] wr_data;
  bit [7:0] exp_rd_data;

  function void display(string tag = "TRANS");
    $display("[%s] EN=%0b WR=%0b RD=%0b WR_ADDR=%0h RD_ADDR=%0h WR_DATA=%0h EXP_RD=%0h",
             tag, enable, wr, rd, wr_addr, rd_addr, wr_data, exp_rd_data);
  endfunction
endclass

// ------------------------------------
// Generator
// ------------------------------------
class dp_generator;
  mailbox #(dp_transaction) gen2drv;
  int count;

  function new(mailbox #(dp_transaction) m);
    gen2drv = m;
  endfunction

  task run();
    dp_transaction tr;
    repeat (count) begin
      tr = new();
      if (!tr.randomize()) $fatal("Randomization failed.");
      gen2drv.put(tr);
    end
  endtask
endclass

// ------------------------------------
// Driver
// ------------------------------------
class dp_driver;
  virtual dp_if vif;
  mailbox #(dp_transaction) gen2drv;
  dp_transaction tr;

  function new(virtual dp_if vif, mailbox #(dp_transaction) m);
    this.vif = vif;
    this.gen2drv = m;
  endfunction

  task run();
    forever begin
      gen2drv.get(tr);
      vif.enable   <= tr.enable;
      vif.wr       <= tr.wr;
      vif.rd       <= tr.rd;
      vif.wr_addr  <= tr.wr_addr;
      vif.rd_addr  <= tr.rd_addr;
      vif.wr_data  <= tr.wr_data;
      @(posedge vif.clk);
    end
  endtask
endclass

// ------------------------------------
// Monitor
// ------------------------------------
class dp_monitor;
  virtual dp_if vif;
  mailbox #(dp_transaction) mon2scb;

  function new(virtual dp_if vif, mailbox #(dp_transaction) m);
    this.vif = vif;
    this.mon2scb = m;
  endfunction

  task run();
    dp_transaction tr;
    forever begin
      @(posedge vif.clk);
      if (vif.enable && vif.rd) begin
        tr = new();
        tr.rd_addr = vif.rd_addr;
        tr.exp_rd_data = vif.rd_data;
        mon2scb.put(tr);
      end
    end
  endtask
endclass

// ------------------------------------
// Scoreboard
// ------------------------------------
class dp_scoreboard;
  mailbox #(dp_transaction) mon2scb;
  bit [7:0] model_mem[0:63];

  function new(mailbox #(dp_transaction) m);
    this.mon2scb = m;
  endfunction

  task run();
    dp_transaction tr;
    forever begin
      mon2scb.get(tr);
      if (tr.rd_addr < 64) begin
        if (model_mem[tr.rd_addr] !== tr.exp_rd_data) begin
          $display("[ERROR] Addr %0h: Expected %0h, Got %0h",
                   tr.rd_addr, model_mem[tr.rd_addr], tr.exp_rd_data);
        end else begin
          $display("[PASS] Addr %0h: Data matched (%0h)", tr.rd_addr, tr.exp_rd_data);
        end
      end
    end
  endtask

  task update_model(dp_transaction tr);
    if (tr.enable && tr.wr && tr.wr_addr < 64) begin
      model_mem[tr.wr_addr] = tr.wr_data;
    end
  endtask
endclass

// ------------------------------------
// DUT: Dual-Port RAM
// ----------------------------------
// ------------------------------------
module dp_tb;

  logic clk = 0;
  always #5 clk = ~clk;

  dp_if vif(clk);

  dp_ram dut (
    .clk(clk),
    .rst(vif.rst),
    .enable(vif.enable),
    .wr(vif.wr),
    .rd(vif.rd),
    .wr_addr(vif.wr_addr),
    .rd_addr(vif.rd_addr),
    .wr_data(vif.wr_data),
    .rd_data(vif.rd_data)
  );

  mailbox #(dp_transaction) gen2drv = new();
  mailbox #(dp_transaction) mon2scb = new();

  dp_generator gen;
  dp_driver drv;
  dp_monitor mon;
  dp_scoreboard scb;

  initial begin
    // Reset
    vif.rst = 0;
    vif.enable = 0;
    vif.wr = 0;
    vif.rd = 0;
    vif.wr_addr = 0;
    vif.rd_addr = 0;
    vif.wr_data = 0;

    #20 vif.rst = 1;

    // Create and connect components
    gen = new(gen2drv); gen.count = 20;
    drv = new(vif, gen2drv);
    mon = new(vif, mon2scb);
    scb = new(mon2scb);

    fork
      gen.run();
      drv.run();
      mon.run();
      scb.run();
    join_none

    #1000 $finish;
  end

endmodule
