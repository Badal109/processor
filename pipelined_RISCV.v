////////////////////////////////////////////////////////////// 
// Company: 
// Engineer: 
//      
// Create Date: 31.07.2025 02:23:59
// Design Name: 
// Module Name: RISC__V
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revisiotimescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.07.2025 13:44:28
// Design Name: 
// Module Name: risc_v
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


// Code your design here
// Code your design here
// Code your design here
// Code your design here
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer// 
// Create Date: 19.01.2025 13:29:59
// Design Name: 
// Module Name: RISC_V
// Project Name: : 

// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////prormme counter////

module programe_counter(input clk,
                        input rst,
                        input [31:0]data,
                        output  reg [31:0]pc_out
                       );
  always@(posedge clk or posedge rst)
    begin
      if(rst==1)
        begin
          pc_out <=0;
          
        end
      else
        begin 
          pc_out<=data;
          
        end
    end
    endmodule
    
    
    // Code your design here

//pc+4 counter 
module pcplus4(input [31:0]fromPC,
               output  [31:0] nextPC);
  assign nextPC=fromPC+4;
  
endmodule

  
///////////////////////////////////////adder mux//////////////////////////
module multiplexer2(
        input [31:0]z1,z2,
    input sel2,
        output reg [31:0]out2);
    always@(*)
      begin
        case(sel2)
          1'b0:out2<=z1;
          1'b1:out2<=z2;
        endcase 
      end
  endmodule
   
//instruction memory////////////////////////////////////////////////////////

module instruction_memory(
  input clk,
  input rst,
 // input read,
  
  input [31:0]inst_addr,
 //nput [31:0]inst,
  output   [31:0]instruction_out);
  reg [ 31:0]mem[63:0];

  
  integer i;
    assign instruction_out=mem[inst_addr];
  
  always@(posedge clk or posedge rst )
    begin
      if(rst==1)
      
        begin
        for(i=0;i<64;i=i+1)
        begin
          mem[i]<=32'h00000000;
          end
          
        end
      else
      begin
      //R TYPE
      mem[0]<=32'b00000000000000000000000000000000;///// no operation
      mem[4]<=32'b0000000_11001_10000_000_01101_0110011;     //add x13,x16,x25
      mem[8]<=32'b0100000_00011_01000_000_00101_0110011; //sub x5,x8,x3
      mem[12]<=32'b0000000_00011_00010_111_00001_0110011;    //and x1,x2,x3
      mem[16]<=32'b0000000_00101_00011_110_00100_0110011;//or x4,x3,x5
      //I TYPE
      mem[20]<=32'b000000000011_10101_000_10110_0010011; // addi x22,x21,3
      mem[24]<=32'b000000000001_01000_110_01001_0010011; // ori  x9,x8,1
      //L type
      mem[28]<=32'b000000001111_00101_010_01000_0000011;///lw x8,15(x5)
      mem[32]<=32'b000000000011_00011_010_01001_0000011; /// lw x9,3(x3)
      //s type
      mem[36]<=32'b0000000_01111_00101_010_01100_0100011; // sw x15,12(x5)
      mem[40]<=32'b0000000_01110_00110_010_01010_0100011;   // x14,10(x6)
      //sb type
      mem[44]<=32'h00948662;// beq x9,x9,12;
      
      
      
      end
       
       end
         endmodule

/////////////////////fetch register for pipelingng//////////////
module IF_ID(
    input clk, rst,
    input [31:0] pc_in, instruction_in,
    output reg [31:0] pc_out, instruction_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_out <= 0;
            instruction_out <= 0;
        end else begin
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
    end
endmodule
                
      //////////////////////////////// end of fetch register/////////////



 ///////////////        ////register mem///////////////////////////
        module register(input clk,
         input rst,
         input regwrite,
         input [4:0]regs1,
         input [4:0]regs2,
         input [4:0]write_reg,
         input [31:0]write_data,
         output [31:0] read_data1,
         output [31:0]read_data2);
   reg [31:0]regs[31:0];
    integer i;
    initial begin
    regs[0]=0;
    regs[1]=4;
    regs[2]=2;
    regs[3]=12;
    regs[4]=4;
    regs[5]=1;
    regs[6]=44;
    regs[7]=4;
    regs[8]=2;
    regs[9]=1;
    regs[10]=23;
    regs[11]=4;
    regs[12]=90;
      regs[13]=10;
      regs[14]=20;
      regs[15]=30;
      regs[16]=46;
      regs[17]=50;
      regs[18]=60;
      regs[19]=70;
      regs[20]=80;
      regs[21]=80;
      regs[22]=90;
      regs[23]=70;
      regs[24]=60;
      regs[25]=6;
      regs[26]=4;
      regs[27]=32;
      regs[28]=12;
      regs[29]=34;
      regs[30]=5;
      regs[31]=10;
      
      
    end
   
         always@(posedge clk )
         begin
           if(rst==1)
             begin
               for(i=0;i<32;i=i+1)
begin
  regs[i]<=32'b00;
  
end
             end
           else
             if(regwrite==1)

             begin
               regs[write_reg]<=write_data;
               
               
             end
         end
   assign read_data1=regs[regs1];
   assign read_data2=regs[regs2];
 endmodule
/////////////////////////////////////////////////immidiate generator/////////////////////////////////
         module immidiate_gen(input [6:0]opcode ,
         input  [31:0]instruction,
         output reg  [31:0]  immext);
         always@(*)
         begin
         case(opcode)
         7'b0000011:
         immext<={ {20{instruction[31]}},instruction[31:20]};
         7'b0100011:
         immext<={{20{instruction[31]}},instruction[31:25],instruction[11:7]};
         7'b1100011:
         immext<={{19{instruction[31]}},instruction[31],instruction[30:25],instruction[11:8],1'b0};
         
         endcase
         end
         endmodule
         
        ////////////////////////control unit/////////////////////
       
        module control_unit(
          input [6:0]instruction,
          output  reg branch,
          output  reg memread,
          output  reg memtoreg ,
          output reg [1:0] ALUOp,
          output reg  memwrite,
          output reg  ALUSrc,
          output reg  regwrite );
          always@(*)
            begin
              case(instruction)
           7'b0110011: {ALUSrc,memtoreg,regwrite,memread,memwrite,branch,ALUOp[1],ALUOp[0]}<=8'b001000_10;
           7'b0000011: {ALUSrc,memtoreg,regwrite,memread,memwrite,branch,ALUOp[1],ALUOp[0]}<=8'b111100_00;
           7'b0100011: {ALUSrc,memtoreg,regwrite,memread,memwrite,branch,ALUOp[1],ALUOp[0]}<=8'b100010_00;
           7'b1100011: {ALUSrc,memtoreg,regwrite,memread,memwrite,branch,ALUOp[1],ALUOp[0]}<=8'b000001_01;
                     
              endcase
             
            end
        endmodule 
        ////////////////////////////////////////////////////////idex reg///////////////////////
        module ID_EX (
            input clk, input rst,
            // Control signals
            input regwrite_in, memtoreg_in, memread_in, memwrite_in,
            input branch_in, ALUsrc_in,
            input [1:0] ALUop_in,
        
            // Data
            input [31:0] pc_in,
            input [31:0] read_data1_in, read_data2_in,
            input [31:0] imm_in,
            input [4:0] rd_in,
            input [2:0] funct3_in,
            input funct7_in,
        
            // Outputs
            output reg regwrite_out, memtoreg_out, memread_out, memwrite_out,
            output reg branch_out, ALUsrc_out,
            output reg [1:0] ALUop_out,
        
            output reg [31:0] pc_out,
            output reg [31:0] read_data1_out, read_data2_out,
            output reg [31:0] imm_out,
            output reg [4:0] rd_out,
            output reg [2:0] funct3_out,
            output reg funct7_out
        );
          always @(posedge clk or posedge rst) begin
            if (rst) begin
              regwrite_out <= 0; memtoreg_out <= 0; memread_out <= 0; memwrite_out <= 0;
              branch_out <= 0; ALUsrc_out <= 0; ALUop_out <= 0;
              pc_out <= 0; read_data1_out <= 0; read_data2_out <= 0;
              imm_out <= 0; rd_out <= 0;
              funct3_out <= 0; funct7_out <= 0;
            end else begin
              regwrite_out <= regwrite_in; memtoreg_out <= memtoreg_in;
              memread_out <= memread_in; memwrite_out <= memwrite_in;
              branch_out <= branch_in; ALUsrc_out <= ALUsrc_in; ALUop_out <= ALUop_in;
              pc_out <= pc_in;
              read_data1_out <= read_data1_in;
              read_data2_out <= read_data2_in;
              imm_out <= imm_in;
              rd_out <= rd_in;
              funct3_out <= funct3_in;
              funct7_out <= funct7_in;
            end
          end
        endmodule

        ////////////////////////////////////////////////////////////////////////////////////
       
//////////// 32 bit ALU////////////
module ALU(
input [31:0]a,
input [31:0]b,
  input [3:0]ALU_command,
  output reg zero,
  output reg [63:0]ALU_out,
output ALU_carry1,ALU_carry2,ALU_carry3);
  wire [63:0]mul_out ;
reg [31:0]temp;
wire[31:0] add,add_with_carry,sub,shift_right ,shift_left,logical_and,logical_or;

parameter addition=4'b0000;
parameter addition_with_carry=4'b0001;
parameter subtraction=4'b0010;
parameter multiplication=4'b0011;
parameter shift_R=4'b0100;
parameter shift_L=4'b0101;
parameter bitwise_or=4'b0110;
parameter bitwise_and=4'b0111;
carry_look_ahead A0(a,b,1'b0,add,ALU_carry1);
carry_look_ahead A1(a,b,1'b1,add_with_carry,ALU_carry2);
carry_look_ahead A2(a,~b+1,1'b0,sub,ALU_carry3);
bit32_booth_multiplier M1(a,b,mul_out);
///shift module
left_shift_ls L0(a,shift_left);
right_shift_rs L1(a,shift_right);

///and module
and_32 A(a,b,logical_and);

  or_32 B(a,b,logical_or);
///////now institae the module////

always@(*)
begin
case(ALU_command)

addition: ALU_out=add;
addition_with_carry: ALU_out=add_with_carry;
subtraction: 
  begin
    if(a==b)
      begin
        zero<=1'b1;
      end
    else
      zero<=0;
 ALU_out=sub;
  end

multiplication: ALU_out=mul_out;
shift_R: ALU_out=shift_right;
shift_L: ALU_out=shift_left;
bitwise_or: ALU_out=logical_or;
bitwise_and: ALU_out=logical_and;
default: 
  begin
    zero<=0;
     ALU_out=64'bx;
  end
 
endcase

end

endmodule

//module forr carry look ahead adder
module carry_look_ahead(input [31:0]a,
input [31:0]b,
input cin,
output [31:0]sum,
output cout);
wire c1,c2,c3,c4,c5,c6,c7;
carry_look_4bits C0(a[3:0],b[3:0],cin,sum[3:0],c1);
carry_look_4bits C1(a[7:4],b[7:4],c1,sum[7:4],c2);
carry_look_4bits C2(a[11:8],b[11:8],c2,sum[11:8],c3);
carry_look_4bits C3(a[15:12],b[15:12],c3,sum[15:12],c4);
carry_look_4bits C4(a[19:16],b[19:16],c4,sum[19:16],c5);
carry_look_4bits C5(a[23:20],b[23:20],c5,sum[23:20],c6);
carry_look_4bits C6(a[27:24],b[27:24],c6,sum[27:24],c7);
carry_look_4bits C7(a[31:28],b[31:28],c7,sum[31:28],cout);
endmodule
///now for 4 bit carry look ahahesd adder 
module carry_look_4bits(input [3:0]a,
input [3:0]b,
input cin,
output [3:0]sum,
output cout);
wire [3:0]c;
wire [3:0]p,g;
assign p=a^b;//propogate 

assign g=a&b;//genenrate

assign c[0]=cin;
assign c[1]=g[0]|(p[0]&c[0]);
assign c[2]=g[1]|(p[1]&g[0])|(p[1]&p[0]&c[0]);
assign c[3]=g[2]|(p[2]&g[1])|(p[1]&p[2]&g[0])|(p[1]&p[0]&p[2]&c[0]);
assign cout=g[3]|(p[3]&g[2])|(p[3]&p[2]&g[1])|(p[1]&p[2]&p[3]&g[0])|(p[0]&p[1]&p[2]&p[3]&c[0]);
assign sum=p^c;
endmodule


//module for 32bits _booth multipler
module bit32_booth_multiplier(input [31:0]a,
input [31:0]b,
output reg [63:0]out);
reg [31:0]accumulator;
reg [31:0]A_complement;
reg [64:0]temp_out;
reg q0;
integer i;
always@(*)
begin
A_complement=~a+1;
q0=0;
accumulator=32'd0;
temp_out={accumulator[31:0],b,q0};
for(i=0;i<32;i=i+1)
begin
case(temp_out[1:0])
2'b00:temp_out={temp_out[64],temp_out[64:1]};
2'b11:temp_out={temp_out[64],temp_out[64:1]};     

///temp_out[64]=Qn;
2'b01:
begin
temp_out[64:33]=temp_out[64:33]+a;
temp_out={temp_out[64],temp_out[64:1]};

end


2'b10: begin
temp_out[64:33]=temp_out[64:33]+A_complement;
temp_out={temp_out[64],temp_out[64:1]};

end

default:temp_out=65'b0;
endcase
case(temp_out[64])
1'b0:out=temp_out[64:1];
1'b1:out=temp_out[64:1]+1;
endcase 

end


end

endmodule
module left_shift_ls(input [31:0]a,
output [31:0]shift_left
);
assign shift_left=a<<1;
endmodule

module right_shift_rs(input[31:0]a,
output [31:0]shift_right 
);
assign shift_right=a>>1;
endmodule

module and_32(input [31:0]a,
input [31:0]b,
              output [31:0]logical_and);
assign logical_and=a&b;
endmodule
module or_32(input [31:0]a,
input [31:0]b,
             output [31:0]logical_or);
assign logical_or=a|b;
endmodule
////////////////////alu mux/////////////////////////////
  module multiplexer0(
  input [31:0]I1,I2,
  input sel0,
  output reg [31:0]out0);
  always@(*)
    begin
      case(sel0)
        1'b0:out0<=I1;
        1'b1:out0<=I2;
      endcase 
    end
    endmodule
////////////////////////////////////////ALU CONTROL LOGIC//////////////////////////////////////////
module ALU_control(
  
  input [1:0]ALUOp,
  input func7,
  input [2:0]func3,
  output reg [3:0]operation);
  always@(*)
    begin
      case({ALUOp,func3,func7})
        6'b00_0_000:operation<=4'b0010;
         6'b01_0_000:operation<=4'b0110;
         6'b10_0_000:operation<=4'b0010;
         6'b10_1_000:operation<=4'b0110;
         6'b10_0_111:operation<=4'b0000;
         6'b10_0_110:operation<=4'b0001;
      endcase 
        
        
        
    end
endmodule
/////////////////////////////////
  
////////////////////////////// adder////////////////////////////////////////
module adder(input [31:0]a1,
           input [31:0]a2,
           output [31:0]sum_out);
assign sum_out=a1+a2;
endmodule
//////////////////////////////////////// exmem register
module EX_MEM (
    input clk, input rst,
    // Control
    input regwrite_in, memtoreg_in, memread_in, memwrite_in,
    input branch_in,
    // Data
    input zero_in,
    input [31:0] alu_result_in,
    input [31:0] write_data_in,
    input [4:0] rd_in,
    input [31:0] pc_branch_in,

    // Outputs
    output reg regwrite_out, memtoreg_out, memread_out, memwrite_out,
    output reg branch_out,
    output reg zero_out,
    output reg [31:0] alu_result_out,
    output reg [31:0] write_data_out,
    output reg [4:0] rd_out,
    output reg [31:0] pc_branch_out
);
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      regwrite_out <= 0; memtoreg_out <= 0; memread_out <= 0;
      memwrite_out <= 0; branch_out <= 0;
      zero_out <= 0; alu_result_out <= 0;
      write_data_out <= 0; rd_out <= 0; pc_branch_out <= 0;
    end else begin
      regwrite_out <= regwrite_in; memtoreg_out <= memtoreg_in;
      memread_out <= memread_in; memwrite_out <= memwrite_in;
      branch_out <= branch_in; zero_out <= zero_in;
      alu_result_out <= alu_result_in;
      write_data_out <= write_data_in;
      rd_out <= rd_in; pc_branch_out <= pc_branch_in;
    end
  end
endmodule


//////////////////////////////////////////////////data memory////////////////////////////////////////////
  module data_memory(
  input clk,
  input rst,
  input read,
  input write ,
  input [31:0]addr,
  input [31:0]data,
  output reg [31:0]data_out);
  reg [31:0]mem[63:0];
  integer i;
  always@(posedge clk or posedge rst)
    begin///
      if(rst==1)
        for(i=0;i<64;i=i+1)
          
        begin
          mem[i]<=32'b00;
          data_out<=32'b00;
        end
      
      else 
        begin
              if(read==1)
                begin
                  data_out<=mem[addr];
                  
                end
              else
                if(write==1)
                  
                begin
                  mem[addr]<=data;
                end
              
              end
        end
endmodule

////////////////////AND//////////////
  module logical_and(input branch,
  input zero,
  output and_out);
  assign and_out=branch&zero;
  endmodule

////////////////////////////////////mem write back reg ////////////////////////////////////////////////
module MEM_WB (
    input clk, input rst,
    input regwrite_in, memtoreg_in,
    input [31:0] mem_data_in,
    input [31:0] alu_result_in,
    input [4:0] rd_in,

    output reg regwrite_out, memtoreg_out,
    output reg [31:0] mem_data_out,
    output reg [31:0] alu_result_out,
    output reg [4:0] rd_out
);
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      regwrite_out <= 0; memtoreg_out <= 0;
      mem_data_out <= 0; alu_result_out <= 0; rd_out <= 0;
    end else begin
      regwrite_out <= regwrite_in; memtoreg_out <= memtoreg_in;
      mem_data_out <= mem_data_in;
      alu_result_out <= alu_result_in;
      rd_out <= rd_in;
    end
  end
endmodule

  ////////////////////////////////////////////////multiplexer/////////////////////////////////////////

    ///////////////MUX2//////////////////////
    module multiplexer1(
    input [31:0]IN1,IN2,
    input sel1,
      output reg [31:0]out1);
    always@(*)
      begin
        case(sel1)
          1'b0:out1<=IN1;
          1'b1:out1<=IN2;
        endcase 
      end
      endmodule
      ///////////////////////MUX3///////////////////////



  ////////////////////////all modules instiate here//////////////////////////////////////////
module RISC_V(
  input  clk,
  input rst
  
      );

  wire zero_top;
  
  wire [31:0] pc_top;
  wire [31:0]ins_top;
  wire regwrite_top,branch_top,memread_top,memtoreg_top;
  wire ALUSrc_top,memwrite_top;
  wire [31:0]immext_top;
  wire [1:0]ALUop_top;
  wire [31:0]ALUMUX_top;
  wire ALUsrc_top;
  wire [3:0]ALUcontrol_top;
  wire [31:0]read_data2_top;
  wire [31:0]read_data1_top;
  wire [31:0]next_PC_top;
  wire [31:0]sum_out_top;
  wire and_out_top;
  wire [31:0]out2_top;
  
  wire [63:0]ALU_out_top;
  wire [31:0]data_out_top;
  wire [31:0]out1_top;
  //////////////////////////////////
  //========== IF/ID ==========
    
  wire [31:0] pc_IFID;
  wire [31:0] ins_IFID;
  
  //========== ID/EX ==========
  wire        regwrite_IDEX;
  wire        memtoreg_IDEX;
  wire        memread_IDEX;
  wire        memwrite_IDEX;
  wire        branch_IDEX;
  wire        ALUsrc_IDEX;
  wire [1:0]  ALUop_IDEX;
  wire [31:0] pc_IDEX;
  wire [31:0] read_data1_IDEX;
  wire [31:0] read_data2_IDEX;
  wire [31:0] imm_IDEX;
  wire [4:0]  rd_IDEX;
  wire [2:0]  funct3_IDEX;
  wire        funct7_IDEX;
  
  //========== EX/MEM ==========
  wire        regwrite_EXMEM;
  wire        memtoreg_EXMEM;
  wire        memread_EXMEM;
  wire        memwrite_EXMEM;
  wire        branch_EXMEM;
  wire        zero_EXMEM;
  wire [31:0] alu_result_EXMEM;
  wire [31:0] write_data_EXMEM;
  wire [31:0] pc_branch_EXMEM;
  wire [4:0]  rd_EXMEM;
  
  //========== MEM/WB ==========
  wire        regwrite_MEMWB;
  wire        memtoreg_MEMWB;
  wire [31:0] mem_data_MEMWB;
  wire [31:0] alu_result_MEMWB;
  wire [4:0]  rd_MEMWB;
///////////////out/////////
wire [31:0]writeback_data;
  //////////////////////////////
    //progrmae counter 
  programe_counter PC(.clk(clk),.rst(rst),.data(out2_top),.pc_out(pc_top));
    //pc+4 counter 
  pcplus4 PC1(.fromPC(pc_top),.nextPC(next_PC_top));
  
  
   multiplexer2 ADD_MUX(.z1(next_PC_top),.z2(sum_out_top),.sel2(and_out_top),.out2(out2_top));//ADD MUX
  
  
    //INSTRUCTION MEMORY 
  instruction_memory M0 (.clk(clk),.rst(rst),. inst_addr(pc_top),.instruction_out(ins_top));
  /////IFID register///////////////////////////////////////
  
  
IF_ID IFID (
    .clk(clk), .rst(rst),
    .pc_in(pc_top),
    .instruction_in(ins_top),
    .pc_out(pc_IFID),
    .instruction_out(ins_IFID)
  );

  

    ///registrer memory
  register R0(.clk(clk),.rst(rst),.regwrite(regwrite_MEMWB),.regs1(ins_top[19:15]),.regs2(ins_top[24:20]),.write_reg(rd_MEMWB),.write_data(writeback_data),.read_data1(read_data1_top),.read_data2(read_data2_top));
    //immidaite generator
  immidiate_gen I0(.opcode(ins_top[6:0]),.instruction(ins_top),.immext(immext_top));
    //control unit
  control_unit C0(.instruction(ins_top[6:0]),.branch(branch_top),.memread(memread_top),.memtoreg(memtoreg_top),.ALUOp(ALUop_top),.memwrite(memwrite_top),.ALUSrc(ALUsrc_top),.regwrite(regwrite_top));
  
  
  /////////////////////////////////insruction decode execute reg/////////////////////
  ID_EX IDEX (
    .clk(clk), .rst(rst),
  
    .regwrite_in(regwrite_top),
    .memtoreg_in(memtoreg_top),
    .memread_in(memread_top),
    .memwrite_in(memwrite_top),
    .branch_in(branch_top),
    .ALUsrc_in(ALUsrc_top),
    .ALUop_in(ALUop_top),
  
    .pc_in(pc_IFID),
    .read_data1_in(read_data1_top),
    .read_data2_in(read_data2_top),
    .imm_in(immext_top),
    .rd_in(ins_IFID[11:7]),
    .funct3_in(ins_IFID[14:12]),
    .funct7_in(ins_IFID[30]),
  
    .regwrite_out(regwrite_IDEX),
    .memtoreg_out(memtoreg_IDEX),
    .memread_out(memread_IDEX),
    .memwrite_out(memwrite_IDEX),
    .branch_out(branch_IDEX),
    .ALUsrc_out(ALUsrc_IDEX),
    .ALUop_out(ALUop_IDEX),
  
    .pc_out(pc_IDEX),
    .read_data1_out(read_data1_IDEX),
    .read_data2_out(read_data2_IDEX),
    .imm_out(imm_IDEX),
    .rd_out(rd_IDEX),
    .funct3_out(funct3_IDEX),
    .funct7_out(funct7_IDEX)
  );

  //////////////////////////////////////////////////////////////
    //32 bit alu
  ALU A0(.a(read_data1_top),.b(ALUMUX_top),.ALU_command(ALUcontrol_top),.zero(zero_top),.ALU_out(ALU_out_top),.ALU_carry1(ALU_carry1),.ALU_carry2(ALU_carry2),.ALU_carry3(ALU_carry3));
  
  //alu control logic
  ALU_control AL0(.ALUOp(ALUop_top),.func7(ins_top[30]),.func3(ins_top[14:12]),.operation(ALUcontrol_top));
  
    multiplexer0 ALU_MUX(.I1(read_data2_top),.I2(immext_top),.sel0(ALUsrc_top),.out0(ALUMUX_top));//ALU MUX
      adder AD0(.a1(pc_top),.a2(immext_top),.sum_out(sum_out_top));////adder 
      EX_MEM EXMEM (
        .clk(clk), .rst(rst),
      
        .regwrite_in(regwrite_IDEX),
        .memtoreg_in(memtoreg_IDEX),
        .memread_in(memread_IDEX),
        .memwrite_in(memwrite_IDEX),
        .branch_in(branch_IDEX),
        .zero_in(zero_top),
        .alu_result_in(ALU_out_top[31:0]),
        .write_data_in(read_data2_IDEX),
        .rd_in(rd_IDEX),
        .pc_branch_in(sum_out_top),
      
        .regwrite_out(regwrite_EXMEM),
        .memtoreg_out(memtoreg_EXMEM),
        .memread_out(memread_EXMEM),
        .memwrite_out(memwrite_EXMEM),
        .branch_out(branch_EXMEM),
        .zero_out(zero_EXMEM),
        .alu_result_out(alu_result_EXMEM),
        .write_data_out(write_data_EXMEM),
        .rd_out(rd_EXMEM),
        .pc_branch_out(pc_branch_EXMEM)
      );

    //data memory
  data_memory DM0(.clk(clk),.rst(rst),.read(memread_top),.write(memwrite_top),.addr(ALU_out_top),.data(read_data2_top),.data_out(data_out_top));
    logical_and AND0(.branch(branch_top),.zero(zero_top),.and_out(and_out_top));/////and
    //////////////////////////mem write back reg
    MEM_WB MEMWB (
      .clk(clk), .rst(rst),
      .regwrite_in(regwrite_EXMEM),
      .memtoreg_in(memtoreg_EXMEM),
      .mem_data_in(data_out_top),
      .alu_result_in(alu_result_EXMEM),
      .rd_in(rd_EXMEM),
    
      .regwrite_out(regwrite_MEMWB),
      .memtoreg_out(memtoreg_MEMWB),
      .mem_data_out(mem_data_MEMWB),
      .alu_result_out(alu_result_MEMWB),
      .rd_out(rd_MEMWB)
    );

  
  
  multiplexer1 DATA_MUX(.IN1(alu_result_MEMWB),.IN2(mem_data_MEMWB),.sel1(memtoreg_MEMWB),.out1(writeback_data));//DATA MEMORY MUX
  
 


  
  endmodule
  
  
