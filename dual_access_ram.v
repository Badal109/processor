// Code your design here
module dp_ram(input clk,
             input rst ,
             input enable,
             input wr,
             input rd,
              input [5:0]wr_addr,
              input [5:0]rd_addr,
              input [7:0]wr_data,
              output reg [7:0]rd_data);
  integer i;
  reg [7:0]mem[63:0];
  
  always@(posedge clk)
    begin
      if(rst==0)begin
        for(i=0;i<=63;i++)begin
          mem[i]<=8'bx;
          rd_data<=8'bx;
                          end
                 end
      else
        begin
          if(enable==1)
            begin
              if(wr==1&&rd==0)begin
                mem[wr_addr]<=wr_data;
                  
                end
              else if(wr==0&&rd==1)begin
                rd_data<=mem[rd_addr];
                
              end
              else if(wr==1&&rd==1)begin
                mem[wr_addr]<=wr_data;
                 rd_data<=mem[rd_addr];
              end
              else
                begin
                  
                   for(i=0;i<=63;i++)begin
                     mem[i]<=mem[i];
                     
          
                          end
                end
              end
          else
            begin
               for(i=0;i<=63;i++)begin
                 mem[i]<=mem[i];
          
                          end
            end
            end
        
      
      
      
      
      
      
      
      
      
      
      
    end
endmodule 

