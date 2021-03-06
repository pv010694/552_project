module cpu_tb();

reg clk,rst_n;

wire [15:0] pc;

//////////////////////
// Instantiate CPU //
////////////////////
cpu_cache_pipelined_version iCPU(.clk(clk), .rst_n(rst_n), .mem_wb_hlt(hlt), .pc(pc));

initial begin
  clk = 0;
  $display("rst assert\n");
  rst_n = 0;
  @(posedge clk);
  @(negedge clk);
  rst_n = 1;
  
  $display("rst deassert\n");
end 
  
always
  #1 clk = ~clk;
  
initial begin
  @(posedge hlt);
  @(posedge clk);
  $stop();
end  

endmodule
