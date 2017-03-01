module wiscf15_alu(src0, src1, shft, func, result ,v,z,n);


input [15:0] src0,src1;
input [3:0] shft;
input [3:0] func;
reg [15:0] src1_new;

output v,z,n;  

//Register to hold the flags

reg v,z,n;    		    

  
output reg [15:0] result; 

reg v1;
 
  
reg [15:0] intermediate1,intermediate2,intermediate3;
  
  
parameter 
ADD=4'b0000,
PADDSB=4'b0001,
SUB=4'b0010,
NAND=4'b0011,
XOR=4'b0100,
SLL=4'b0101,
SRL=4'b0110,
SRA=4'b0111,
HLT= 4'b1111,
LW=4'b1000,
SW=4'b1001;

 
always @(*)
begin
//////////////////////////////////////////////////////
///Default values to prevent unintended latches///////
/////////////////////////////////////////////////////
n=0; z=0; v=0;
result=16'h0000;
intermediate1=16'h0000;
intermediate2=16'h0000;
intermediate3=16'h0000;

case (func)


ADD : begin 
		
          result = src0 + src1;   
	  v=(~result[15]&&src0[15]&&src1[15])||(result[15]&&~src0[15]&&~src1[15]); //overflow flag is set
          if(v)
	  begin
	  if(src0[15])
	  result=16'h8000;    //negative saturation
	  else
	  result=16'h7fff;    //positive saturation
	  end

	  if(result==16'h0000)
	  z=1;			//setting zero flag
	  

	  if(result[15]==1)
          n=1;			//setting sign bit
	end
          
          
	
PADDSB : begin 
		result[7:0]=src1[7:0]+src0[7:0];
		v1=(~result[7]&&src0[7]&&src1[7])||(result[7]&&~src0[7]&&~src1[7]);
		if(v1)
	 	 begin
		  if(src0[7])
	  	result[7:0]=8'h80;		//negative saturation
	 	 else
	  	result[7:0]=8'h7f;		//positive saturation
	  	end

		result[15:8]=src1[15:8]+src0[15:8];
		v1=(~result[15]&&src0[15]&&src1[15])||(result[15]&&~src0[15]&&~src1[15]);
		if(v1)
	 	 begin
		  if(src0[15])
	  	result[15:8]=8'h80;		//negative saturation
	 	 else
	  	result[15:8]=8'h7f;		//positive saturation
	  	end
	end
  		 
SUB : begin
		src1_new=~src1+1;
		result = src0 + src1_new;   
	 	 v=(~result[15]&&src0[15]&&src1_new[15])||(result[15]&&~src0[15]&&~src1_new[15]);	//overflow flag
         	 if(v)
	  	begin
	 	 if(src0[15])
	  	result=16'h8000;		//negative saturation
	  	else
	  	result=16'h7fff;		//positive saturation
	  	end


		if(result==16'h0000)
	        z=1;				//zero flag
		else
		z=0;
	  

	       if(result[15]==1'b1)
                n=1;				//sign flag
		else
		n=0;
	end
NAND : begin
		result = ~(src0 & src1);
		if(result==16'h0000)
	        z=1;				//zero flag

	end

XOR: begin
		result=src0 ^ src1;
		if(result==16'h0000)
	        z=1;				//zero flag
     end

SLL: begin	 intermediate1 = ( shft[0]) ?  {src0[14:0],1'b0} : src0;
		 intermediate2 = (shft [1]) ?  { intermediate1 [13:0],2'b00} : intermediate1;
		 intermediate3 = (shft [2]) ?  {intermediate2[11:0], 4'h0} :intermediate2;
		 result = (shft[3])  ? {intermediate3[7:0],8'h00} : intermediate3;
		if(result==16'h0000)
	        z=1;				//zero flag
		end
		
SRL: begin
		 intermediate1 = ( shft[0]) ?  {1'b0,src0[15:1]} : src0;
		 intermediate2 = (shft [1]) ?  { 2'b00,intermediate1 [15:2]} : intermediate1;
		 intermediate3 = (shft[2]) ?  {4'h0,intermediate2[15:4] } :intermediate2;
		 result = (shft[3])  ? {8'h00,intermediate3[15:8]} : intermediate3;
		if(result==16'h0000)
	        z=1;				//zero flag
		 end
SRA : begin
 		  intermediate1 = ( shft[0]) ?  {src0[15],src0[15:1]} : src0;
		 intermediate2 = (shft [1]) ?  { {2{intermediate1[15]}},intermediate1 [15:2]} : intermediate1;
		 intermediate3 = (shft [2]) ?  {{4{intermediate2[15]}},intermediate2[15:4] } :intermediate2;
		 result = (shft[3])  ? {{8{intermediate3[15]}},intermediate3[15:8]} : intermediate3;
		if(result==16'h0000)
	        z=1;				//zero flag
		end

LW: begin
	    result=src0+src1; //base register + offset
    end

SW: begin
		result=src0+src1;	//base register+offset
     end

/*HLT: begin
    		
		hlt=1'b1;		//halt
     end	*/	

default: result=16'h0000;
endcase
		 
end
endmodule
