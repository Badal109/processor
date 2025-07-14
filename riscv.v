// Code your design here
// Code your design here
// Code your design here
// Code your design here
// Code your design here
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.01.2025 13:29:59
// Design Name: 
// Module Name: RISC_V
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
               output[31:0] nextPC);
  assign nextPC=fromPC+4;
  
endmodule
   
//instruction memory////////////////////////////////////////////////////////

module instruction_memory(
  input clk,
  input rst,
 // input read,
  
  input [31:0]inst_addr,
 //nput [31:0]inst,
  output    [31:0]instruction_out);
  reg [ 31:0]mem[63:0];
  assign instruction_out=mem[inst_addr];
  
  integer i;
  
  
  always@(posedge clk or posedge rst )
    begin
      if(rst==1)
      
        begin
        for(i=0;i<64;i=i+1)
        begin
          mem[i]<=32'b00;
          end
          
        end
      else
      begin
      //R TYPE
      mem[0]=32'b00000000000000000000000000000000;///// no operation
      mem[4]=32'b0000000_11001_10000_000_01101_0110011;     //add x13,x16,x25
      mem[8]=32'b0100000_00011_01000_000_00101_0110011; //sub x5,x8,x3
      mem[12]=32'b0000000_00011_00010_111_00001_0110011;    //and x1,x2,x3
      mem[16]=32'b0000000_00101_00011_110_00100_0110011;//or x4,x3,x5
      //I TYPE
      mem[20]=32'b000000000011_10101_000_10110_0010011; // addi x22,x21,3
      mem[24]=32'b000000000001_01000_110_01001_0010011; // ori  x9,x8,1
      //L type
      mem[28]=32'b000000001111_00101_010_01000_0000011;///lw x8,15(x5)
      mem[32]=32'b000000000011_00011_010_01001_0000011; /// lw x9,3(x3)
      //s type
      mem[36]=32'b0000000_01111_00101_010_01100_0100011; // sw x15,12(x5)
      mem[40]=32'b0000000_01110_00110_010_01010_0100011;   // x14,10(x6)
      //sb type
      mem[44]=32'h00948662;// beq x9,x9,12;
      
      
      end
      
       end
         endmodule
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
    regs[3]=24;
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
      regs[16]=40;
      regs[17]=50;
      regs[18]=60;
      regs[19]=70;
      regs[20]=80;
      regs[21]=80;
      regs[22]=90;
      regs[23]=70;
      regs[24]=60;
      regs[25]=65;
      regs[26]=4;
      regs[27]=32;
      regs[28]=12;
      regs[29]=34;
      regs[30]=5;
      regs[31]=10;
      
      
    end
   
         always@(posedge clk or posedge rst)
         begin
           if(rst==1)
             begin
               for(i=0;i<64;i=i+1)
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
carry_look_ahead A0(a,b,0,add,ALU_carry1);
carry_look_ahead A1(a,b,1,add_with_carry,ALU_carry2);
  carry_look_ahead A2(a,~b+1,0,sub,ALU_carry3);
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
  ////////////////////////////////////////////////multiplexer/////////////////////////////////////////
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
////////////////////AND//////////////
  module logical_and(input branch,
  input zero,
  output and_out);
  assign and_out=branch&zero;
  endmodule
  
////////////////////////////// adder////////////////////////////////////////
module adder(input [31:0]a1,
             input [31:0]a2,
             output [31:0]sum_out);
  assign sum_out=a1+a2;
endmodule


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
  
  wire [31:0]ALU_out_top;
  wire [31:0]data_out_top;
  wire [31:0]out1_top;
  
  
  
    //progrmae counter 
  programe_counter PC(.clk(clk),.rst(rst),.data(out2_top),.pc_out(pc_top));
    //pc+4 counter 
  pcplus4 PC1(.fromPC(pc_top),.nextPC(next_PC_top));
    //INSTRUCTION MEMORY 
  instruction_memory M0 (.clk(clk),.rst(rst),. inst_addr(pc_top),.instruction_out(ins_top));
    ///registrer memory
  register R0(.clk(clk),.rst(rst),.regwrite(regwrite_top),.regs1(ins_top[19:15]),.regs2(ins_top[24:20]),.write_reg(ins_top[11:7]),.write_data(out1_top),.read_data1(read_data1_top),.read_data2(read_data2_top));
    //immidaite generator
  immidiate_gen I0(.opcode(ins_top[6:0]),.instruction(ins_top),.immext(immext_top));
    //control unit
  control_unit C0(.instruction(ins_top[6:0]),.branch(branch_top),.memread(memread_top),.memtoreg(memtoreg_top),.ALUOp(ALUop_top),.memwrite(memwrite_top),.ALUSrc(ALUsrc_top),.regwrite(regwrite_top));
    //32 bit alu
  ALU A0(.a(read_data1_top),.b(ALUMUX_top),.ALU_command(ALUcontrol_top),.zero(zero_top).ALU_out(ALU_out_top),.ALU_carry1(ALU_carry1),.ALU_carry2(ALU_carry2),.ALU_carry3(ALU_carry3));
  
  //alu control logic
  ALU_control AL0(.ALUOp(ALUop_top),.func7(ins_top[30]),.func3(ins_top[14:12]),.operation(ALUcontrol_top));
    //data memory
  data_memory DM0(.clk(clk),.rst(rst),.read(memread_top),.write(memwrite_top),.addr(ALU_out_top),.data(read_data2_top),.data_out(data_out_top));
  
  
  multiplexer0 ALU_MUX(.I1(read_data2_top),.I2(immext_top),.sel0(ALUsrc_top),.out0(ALUMUX_top));//ALU MUX
  
  multiplexer1 DATA_MUX(.IN1(ALU_out_top),.IN2(data_out_top),.sel1(memtoreg_top),.out1(out1_top));//DATA MEMORY MUX
  
  multiplexer2 ADD_MUX(.z1(next_PC_top),.z2(sum_out_top),.sel2(and_out_top),.out2(out2_top));//ADD MUX
  
  adder AD0(.a1(pc_top),.a2(immext_top),.sum_out(sum_out_top));////adder 
  logical_and AND0(.branch(branch_top),.zero(zero_top),.and_out(and_out_top));/////and
  
  endmodule
    
// we are only left with alu and control unit

