module cpu_cache_pipelined_version(clk,rst_n,mem_wb_hlt,pc);

localparam 
NE=3'b000,
E=3'b001,
GT=3'b010,
LT=3'b011,
GTE=3'b100,
LTE=3'b101,
OV=3'b110,
UNCOND=3'b111;

typedef enum reg [2:0]  {IDLE,EVICT_DCACHE_R, EVICT_DCACHE_W, READ_UM_R, READ_UM_W, INSTR_RD } state_t;
state_t state, next_state;


input clk,rst_n;
wire hlt;
output reg [15:0] pc;
reg [15:0] return_addr;

//Wires for Instruction memory
reg rd_en;
reg [15:0] instr;

//Wires for Register File
reg [3:0] p0_addr, p1_addr, p0_addr_new, p1_addr_new;
wire [15:0] p0,p1;
wire re0,re1;
reg [3:0] dst_addr,dst_addr_new;
reg [15:0] dst;
reg we, b_cond;
wire lw;
wire [3:0]opcode;
wire [2:0] ccc;
reg call,ret;
reg sw_select;
reg shift, mem, mem_reg, we_dm, rd_data;
reg branch;
reg branch_cond;
reg [3:0] call_addr_new;
wire [15:0] lhb_result,llb_result;

//Wires for ALU
reg [3:0]shift_amt;
reg [15:0] p1_mux;
wire [15:0] result;
wire v,z,n;

//Wires for Data memory
reg re_dm;
reg [15:0] read_data;

wire [15:0] call_address;
wire [15:0] branch_address;

////////////////////////////////////////////////////////////
//////////////////Wires for halt////////////////////////////
////////////////////////////////////////////////////////////
reg id_ex_hlt, ex_mem_hlt;
output reg mem_wb_hlt;



/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
///////////Declaration of pipeline registers///////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

reg [15:0] if_id_instr;

////////////////////////////////////////////////////////
////////////////Pipeline registers-2///////////////////
//////////////////////////////////////////////////////
reg [3:0] id_ex_p1_addr, id_ex_p0_addr, id_ex_p0_addr_new, id_ex_p1_addr_new, id_ex_dst_addr, id_ex_dst_addr_new, id_ex_opcode;
reg [15:0] id_ex_p0,id_ex_p1,id_ex_instr, id_ex_sign_extend ;
reg id_ex_sw_select, id_ex_lhb, id_ex_llb, id_ex_mem, id_ex_mem_reg, id_ex_we_dm;
reg id_ex_re_dm, id_ex_we;
reg id_ex_call;
reg [15:0] id_ex_return;
reg id_ex_shift;
reg [2:0] id_ex_ccc;
reg [15:0] id_ex_branch_address;
reg id_ex_lw, id_ex_ret, id_ex_b_cond;
reg [3:0] id_ex_call_addr_new;

//////////////////////////////////////////////////////////////////////
//////////////////Pipeline registers - 3/////////////////////////////
////////////////////////////////////////////////////////////////////
reg [3:0] ex_mem_dst_addr, ex_mem_dst_addr_new;
reg [3:0] ex_mem_p0_addr_new, ex_mem_p1_addr_new;
reg [15:0] ex_mem_instr, ex_mem_result, ex_mem_p0, ex_mem_p1, ex_mem_return;
reg ex_mem_mem, ex_mem_mem_reg, ex_mem_we_dm, ex_mem_re_dm;
reg ex_mem_we, ex_mem_lhb, ex_mem_llb, ex_mem_call;
reg ex_mem_v, ex_mem_z, ex_mem_n;
reg [3:0] ex_mem_opcode;
reg [2:0] ex_mem_ccc;
reg [15:0] ex_mem_branch_address;
reg [3:0] ex_mem_call_addr_new;
reg [15:0] ex_mem_lhb_result, ex_mem_llb_result;

////////////////////////////////////////////////////////////////
///////////////Pipeline registers-4////////////////////////////
//////////////////////////////////////////////////////////////
reg [3:0] mem_wb_dst_addr, mem_wb_dst_addr_new, mem_wb_opcode;
reg [3:0] mem_wb_p0_addr_new, mem_wb_p1_addr_new;
reg [15:0] mem_wb_instr, mem_wb_read_data, mem_wb_p0, mem_wb_result, mem_wb_return;
reg mem_wb_mem_reg, mem_wb_lhb, mem_wb_llb, mem_wb_call, mem_wb_we; 
reg mem_wb_v, mem_wb_z, mem_wb_n;
reg [3:0] mem_wb_call_addr_new;
reg [15:0] mem_wb_lhb_result, mem_wb_llb_result;
///////////////////////////////////////////////////////////////////
////////////////Registers to flop ALU flags///////////////////////
/////////////////////////////////////////////////////////////////
reg store_v,store_z,store_n;

/////////////////////////////////////////////////////////////////
///////////////Registers for multiplexers///////////////////////
///////////////////////////////////////////////////////////////
reg [15:0] mem_wb_read_mux;
reg [15:0] read_mux;
reg [15:0] p0_mux,p1_mux_2,result_muxed;
reg [15:0] ex_mem_p1_mux;
/////////////////////////////////////////////////////////////////
////////////////////Flush and stall control ////////////////////
///////////////////////////////////////////////////////////////

wire if_id_flush, id_ex_flush, ex_mem_flush,mem_wb_flush;

wire stall;
reg id_raw_stall;
reg cache_stall;

/////////////////////////////////////////////////////////////////////////////
///////////////////Pipeline disable control lines///////////////////////////
///////////////////////////////////////////////////////////////////////////
wire if_id_reg_disable,id_ex_reg_disable,ex_mem_reg_disable,mem_wb_reg_disable;


////////////////////////////////////////////////////////////////////////
////////////////////Registers for forwarding///////////////////////////
//////////////////////////////////////////////////////////////////////

wire ex_forward, mem_forward; 
reg ex_forward_1, ex_forward_2, mem_forward_1, mem_forward_2;

///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
///////////////////Logic for flushing the registers//////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/* IF/ID register is flushed during call, ret and branch instructions. 
   ID/EX register is flushed during branch instruction and if there is a stall inserted due to Load Word data dependency
   EX/MEM register is flushed during branch instruction. Here, branch decision is made in the MEM phase. 
   MEM/WB register is never flushed   */

assign if_id_flush=(call||ret||branch)?1:0;
assign id_ex_flush=(branch||id_raw_stall)?1:0; 
assign ex_mem_flush=(branch)?1:0; 
assign mem_wb_flush=1'b0;

////////////////////////////////////////////////////////////////////////////////
///////////////////////Logic for disabling the pipeline registers//////////////
//////////////////////////////////////////////////////////////////////////////
/* Pipeline registers are disable if there is a cache stall, i.e. the entire pipeline is stalled */
assign if_id_reg_disable=(cache_stall)?1:0;
assign id_ex_reg_disable=(cache_stall)?1:0;
assign ex_mem_reg_disable=(cache_stall)?1:0;
assign mem_wb_reg_disable=(cache_stall)?1:0;


//////////////////////////////////////////////////////////////////////////////
///////////////////////////Input/Outputs for I Cache/////////////////////////
////////////////////////////////////////////////////////////////////////////

wire [63:0] iCache_wrt_data, iCache_rd_data;
wire [13:0] pc_to_cache;
wire iCache_wdirty, iCache_we, iCache_re, iCache_hit, iCache_dirty; 
wire [7:0] iCache_tag_out;
wire [1:0] instr_sel;

//////////////////////////////////////////////////////////////////////////////
///////////////////////////Input/Outputs for D Cache/////////////////////////
////////////////////////////////////////////////////////////////////////////

wire [13:0] addr_to_dcache;
reg [13:0] addr_muxed;
wire [1:0] offset_to_dcache;
reg [63:0] wrt_dcache_from_unified_mem, wrt_dcache_from_pipeline, wrt_dcache;
wire [63:0] read_data_from_dcache;
wire dCache_wdirty;
wire dCache_we, dCache_re;
wire [7:0] dCache_tag_out;
wire dCache_hit, dCache_dirty;

//////////////////////////////////////////////////////////////////////////////
//////////////////////Input/Outputs for Unified memory///////////////////////
////////////////////////////////////////////////////////////////////////////

reg um_we, um_re, im_we, dm_re, dm_we;
reg select_pipeline, select_data_addr;

wire rdy;

////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
//////////////////STATE MACHINE FOR CACHE/////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
/* There are six states in the state machine: IDLE, EVICT_DCACHE_R, EVICT_DCACHE_W, READ_UM_R, READ_UM_W, INSTR_RD.
The state machine is initially in the IDLE state. If a write miss occurs and the block is dirty, the SM goes to EVICT_DCACHE_W state where data
is copied from the data cache into unified mem. Then the SM goes to the READ_UM_W state. If a write miss occurs and the block is not dirty, the SM 
directly goes to READ_UM_W. In READ_UM_W, data is copied from unified_mem and placed in the data cache. In the next cycle data hit will occur and 
the desired block will get written. 

Read misses are handled in a similar manner by the EVICT_DCACHE_R and READ_UM_R states. If data cache miss and instruction cache miss occurs 
simultaneously, the data cache miss is prioritized. After the data cache misses have been handled, the SM checks if there is a instruction cache miss.
If there is an instruction cache miss, the required instruction is fetched from the unified memory and placed in the instruction cache. In the next cycle
instruction cache hit will occur and the program execution will continue. 

As long as the SM is in states other than IDLE, the cache will be stalled. The SM waits from ready signal from the unified memory before moving on to
the next state*/

always@(posedge clk, negedge rst_n)

  if(!rst_n)
  state<=IDLE;
  else
  state<=next_state;


always_comb
begin

//default outputs here
cache_stall=0;
next_state=IDLE;
um_re=1'b1;
um_we=1'b0;
dm_re=1'b1;
dm_we=1'b0;
im_we=1'b0;
select_pipeline=1'b1;
select_data_addr=1'b0;
  case(state)

  IDLE: begin
	
	if( ex_mem_re_dm && !dCache_hit && dCache_dirty )
	begin
	       next_state=EVICT_DCACHE_R;		//read miss and dirty
	       cache_stall=1;
		   um_we=1'b1;
		   select_data_addr=1'b1;
	      	
		
	end
	
	else if(ex_mem_re_dm && !dCache_hit && !dCache_dirty )
	begin
		next_state=READ_UM_R;				//read miss and not dirty
		cache_stall=1;
		select_data_addr=1'b1;
		
		
	end

	else if( ex_mem_we_dm && !dCache_hit && dCache_dirty)
	begin
		next_state=EVICT_DCACHE_W;			//write miss and dirty
		cache_stall=1;
		um_we=1'b1;
		select_data_addr=1'b1;
		
	end

	else if(ex_mem_we_dm && ! dCache_hit && !dCache_dirty)
	begin
		next_state=READ_UM_W;				//write miss and not dirty
		cache_stall=1;
		select_data_addr=1'b1;
		
	end

	else if( ((!ex_mem_we_dm && !ex_mem_we_dm)||dCache_hit) && !iCache_hit)//
	begin
		next_state=INSTR_RD;				//instruction read miss
		cache_stall=1;
		
	end

	else
		next_state=IDLE;					//no miss.
	end

  EVICT_DCACHE_R: begin
		  cache_stall=1;
		  if(!rdy)
			begin
			next_state=EVICT_DCACHE_R;
			select_data_addr=1'b1;
			um_we=1'b1;
			
			end
		  else 
			begin
			next_state=READ_UM_R;
			select_data_addr=1'b1;
			end
		  end

  READ_UM_R:  begin
		
		if(!rdy)
		begin
			next_state=READ_UM_R;
			cache_stall=1;
			select_data_addr=1'b1;
			
		end
		else if(rdy && iCache_hit)
		begin
			next_state=IDLE;
			cache_stall=1;
			select_pipeline=1'b0;
			dm_we=1'b1;
			
		end
		else
		begin
			next_state=INSTR_RD;
			cache_stall=1;
			select_pipeline=1'b0;
			dm_we=1'b1;
		end

	      end

  EVICT_DCACHE_W:  begin
		  cache_stall=1;
		  if(!rdy)
			begin
			next_state=EVICT_DCACHE_W;
			dm_re=1'b1;
			um_we=1'b1;
			select_data_addr=1'b1;
			end
				
		  else 
			begin
			next_state=READ_UM_W;
			select_data_addr=1'b1;//
		  	end	 
		  end		

  READ_UM_W:  begin

		if(!rdy)
		begin
			next_state=READ_UM_W;
			cache_stall=1;
			select_data_addr=1'b1;
			
		end
		else if(rdy && iCache_hit)
		begin
			next_state=IDLE;
			cache_stall=1;
			select_pipeline=1'b0;
			dm_we=1'b1;
		end
		else
		begin
			next_state=INSTR_RD;
			cache_stall=1;
			select_pipeline=1'b0;
			dm_we=1'b1;
		end

	      end

  INSTR_RD:  begin

		if(!rdy)
		begin
			next_state= INSTR_RD;
			cache_stall=1;
			
		end
		else
		begin
			next_state=IDLE;
			cache_stall=1;
			im_we=1'b1;
			
		end
	     end

  default:  begin
		cache_stall=0;
		next_state=IDLE;
		um_re=1'b1;
		um_we=1'b0;
		dm_re=1'b1;
		dm_we=1'b0;
		im_we=1'b0;
		select_pipeline=1'b1;
		select_data_addr=1'b0;//
	    end
		
  endcase
end





//////////////////////////////////////////////////////////////////////
/////////////////PC incrementing logic///////////////////////////////
////////////////////////////////////////////////////////////////////
always@(posedge clk, negedge rst_n)
begin
	if(!rst_n)
		begin
		pc<=16'h0000;
		rd_en<=1;
  		end
	else if(mem_wb_hlt)
		begin
		pc<=pc;
		rd_en<=0;
		end
	else if(cache_stall)		
		pc<=pc;
	else if (branch)
		pc<=pc-2+ex_mem_branch_address;  		//flush if/id,id/ex,ex/mem pipeline reg for this case
	else if(call)
		pc<= pc+call_address;					//flush if/id pipeline reg for this case
	else if(ret)
		pc<=return_addr;						//flush if/id pipeline reg for this case
	else if(stall)
		pc<=pc;
  	else
    	pc<=pc+1;
end

/*Branch decision is made in the MEM phase in this design */


///////////////////////////////////////////////////////////////////////
//////////////////////////Instantiate I CACHE ////////////////////////
/////////////////////////////////////////////////////////////////////

assign iCache_wrt_data=wrt_dcache_from_unified_mem; //iCache is written from Unified memory
assign iCache_wdirty=1'b0;
assign pc_to_cache= pc[15:2];
assign iCache_we=(iCache_hit)?1'b0:im_we;			//Write enable is 0 on iCache hit. Write enable is set by state machine in case of cache miss.
assign instr_sel= pc[1:0];							// two LSB are used as offset to select the instructions

assign iCache_re=1'b1;

cache Instr_Cache(.clk(clk),.rst_n(rst_n),.addr(pc_to_cache),.wr_data(iCache_wrt_data),.wdirty(iCache_wdirty),
                  .we(iCache_we),.re(iCache_re),.rd_data(iCache_rd_data),.tag_out(iCache_tag_out),.hit(iCache_hit),.dirty(iCache_dirty));

always_comb
begin
  if(instr_sel==2'b00)
	instr= iCache_rd_data[15:0];
  else if(instr_sel==2'b01)
	instr= iCache_rd_data[31:16];
  else if(instr_sel==2'b10)
	instr= iCache_rd_data[47:32];
  else
	instr= iCache_rd_data[63:48];
end



////////////////////////////////////////////////////////////////////////
///////////////////////////IF/ID PIPELINE REGISTER/////////////////////
///////////////////////////////////////////////////////////////////////

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		if_id_instr<=0;
	else if(!if_id_reg_disable)				//pipeline reg disable on cache miss
		begin
		if(if_id_flush)						
			if_id_instr<=0;
		else if (!id_raw_stall)
			if_id_instr<=instr;
		end
end


//finding the opcode from instruction
assign opcode=if_id_instr[15:12];
assign ccc=if_id_instr[11:9];

//block for sign extending 12bit to 16bit for call address calculation

assign call_address= {{4{if_id_instr[11]}},if_id_instr[11:0]};

//block for branch address calculation
assign branch_address={{7{if_id_instr[8]}},if_id_instr[8:0]};



////////////////////////////////////////////////////
/////////////////CONTROL LOGIC//////////////////////
////////////////////////////////////////////////////
//control lines-sw_select
always@(*) 
begin
if(opcode==4'b1001)
sw_select=1'b1;
else
sw_select=1'b0;
end

//control logic-re0,re1
assign re0=1;
assign re1=1;

//control lines-llb and lhb
reg lhb,llb;
wire lhb_llb;
assign lhb_llb=opcode[3]&&opcode[1]&&~opcode[2];

always@(*)
begin
lhb=0; llb=0;
if(lhb_llb)
begin
  if(opcode[0])
   llb=1;
  else
   lhb=1;
end
end

//control lines-call and ret


always@(*)
begin
call=0; ret=0;
if(opcode==4'b1101)
call=1;
else if(opcode==4'b1110)
ret=1;
end

assign lw=(opcode==4'b1000)?1:0;

/////////////////////////////////////////////////////
//////////Flopping return address (pc+1) ////////////
//////////////////////////////////////////////////////

always@(posedge clk or negedge rst_n)
begin
if(!rst_n)
return_addr<=16'h0000;
else if(call)
return_addr<=pc;
end

/////////////////////////////////////////////////////////////////////
/////////////////////Control logic for memory///////////////////////
////////////////////////////////////////////////////////////////////

always@(*) //control signal-shift
begin
if((opcode==4'b0110)||(opcode==4'b0111)||(opcode==4'b0101))
shift=1'b1;
else
shift=1'b0;
end

always@(*) //control signals- mem,mem_reg,we_dm
begin
mem=1'b0;
mem_reg=1'b0;
we_dm=1'b0;
re_dm=1'b0;

if(opcode==4'b1000)
begin
mem=1'b1;
mem_reg=1'b1;
re_dm=1'b1;
end

else if(opcode==4'b1001)
begin
mem=1'b1;
we_dm=1'b1;
mem_reg=1'b0; 
end

end

//Control for we

always@(*)
begin
    we=1'b0;
  if(if_id_instr==16'h0000)
  we=1'b0;
  else if(opcode[3]==0)
    we=1'b1;
  else if(call) 
    we=1'b1;
  else if(opcode[3]&&~opcode[2]&&~opcode[1]&&opcode[0]) 
    we=1'b0;
  else if(opcode[3]&&~opcode[2]==1'b1)
    we=1'b1;
 
end

/////////////////////////////////////////////////////////////////////////
////////Finding sign extended offset for memory access//////////////////
////////////////////////////////////////////////////////////////////////
wire [3:0] offset;
wire [15:0] sign_extend;
assign offset=if_id_instr[3:0];
assign sign_extend= {{12{offset[3]}},offset};

assign hlt=(opcode==4'b1111)?1:0;
/////////////////////////////////////////////////////
////////////////////////////////////////////////////
//////////////END  OF CONTROL LOGIC////////////////
//////////////////////////////////////////////////
/////////////////////////////////////////////////
 


////////////////////////////////////////////////////////////////////////////////
//////MULTIPLEXERS TO SELECT p0_addr, p1_addr and dst_addr/////////////////////
//////////////////////////////////////////////////////////////////////////////

always@(*)
begin
        
        if(lhb_llb)
	p0_addr=if_id_instr[11:8];
	else
	p0_addr=if_id_instr[7:4];

	if(sw_select)
	p1_addr=if_id_instr[11:8];
	else
	p1_addr=if_id_instr[3:0];
	
	if(mem_wb_call)
	dst_addr=mem_wb_call_addr_new;
	else
	dst_addr= mem_wb_dst_addr_new;

	
end

always@(*)
b_cond=(opcode==4'b1100)?1:0;

///////////////////////////////////////////////////////////
///////////////////////Block to find dst_addr/////////////
//////////////////////////////////////////////////////////
always@(*)
begin
	dst_addr_new=4'b0000;
	if(call)
	dst_addr_new=4'b1111;
	else if(ret)
	dst_addr_new=4'b0000;
	else if(hlt)
	dst_addr_new=4'b0000;
	else if(sw_select)
	dst_addr_new=4'b0000;
	else if(b_cond)
	dst_addr_new=4'b0000;
	else
	dst_addr_new=if_id_instr[11:8];

	call_addr_new=dst_addr_new;

end
////////////////////////////////////////////////////////////////////////
/////////////////////Block to find p0_addr and p1_addr/////////////////
//////////////////////////////////////////////////////////////////////

always@(*)
begin
        
        if(lhb)
	p0_addr_new=if_id_instr[11:8];
	else if(llb)
	p0_addr_new=4'b0000;
	else if(b_cond)
	p0_addr_new=4'b0000;
	else if(call)
	p0_addr_new=4'b0000;
	else if(hlt)
	p0_addr_new=4'b0000;
	else
	p0_addr_new=if_id_instr[7:4];

	if(sw_select)
	p1_addr_new=if_id_instr[11:8];
	else
	p1_addr_new=if_id_instr[3:0];
end

////////////////////////////////////////////////////////////
//Instantiate RF
rf Reg_File(.clk(clk),.p0_addr(p0_addr),.p1_addr(p1_addr),.p0(p0),.p1(p1),.re0(re0),.re1(re1),.dst_addr(dst_addr),.dst(dst),.we(mem_wb_we),.hlt(mem_wb_hlt));


////////////////////////////////////////////////////////
////////////////ID/EX Pipeline registers///////////////
//////////////////////////////////////////////////////


always@(posedge clk or negedge rst_n)
begin
  	if(!rst_n)
	begin
	id_ex_instr<=0;
	id_ex_p0_addr<=0;
	id_ex_p1_addr<=0;
	id_ex_p0_addr_new<=0;
	id_ex_p1_addr_new<=0;
	id_ex_dst_addr_new<=0;
	id_ex_p0<=0;
	id_ex_p1<=0;
	id_ex_sign_extend<=0;
	id_ex_opcode<=0;
	id_ex_ccc<=0;
	id_ex_sw_select<=0;
	id_ex_lhb<=0;
	id_ex_llb<=0;
	id_ex_shift<=0;
	id_ex_mem<=0;
	id_ex_mem_reg<=0;
	id_ex_we_dm<=0;
	id_ex_re_dm<=0;
	id_ex_we<=0;
	id_ex_call<=0;
	id_ex_return<=0;
	id_ex_hlt<=0;
	id_ex_lw<=0;
	id_ex_ret<=0;
	id_ex_b_cond<=0;
	end

	else if(!id_ex_reg_disable && id_ex_flush)
	begin
	id_ex_instr<=0;
	id_ex_p0_addr<=0;
	id_ex_p1_addr<=0;
	id_ex_p0_addr_new<=0;
	id_ex_p1_addr_new<=0;
	id_ex_dst_addr_new<=0;
	id_ex_p0<=0;
	id_ex_p1<=0;
	id_ex_sign_extend<=0;
	id_ex_opcode<=0;
	id_ex_ccc<=0;
	id_ex_sw_select<=0;
	id_ex_lhb<=0;
	id_ex_llb<=0;
	id_ex_shift<=0;
	id_ex_mem<=0;
	id_ex_mem_reg<=0;
	id_ex_we_dm<=0;
	id_ex_re_dm<=0;
	id_ex_we<=0;
	id_ex_call<=0;
	id_ex_return<=0;
	id_ex_hlt<=0;
	id_ex_lw<=0;
	id_ex_ret<=0;
	id_ex_b_cond<=0;


	end

	else if(!id_ex_reg_disable)

	begin
	id_ex_p0_addr<=p0_addr;
	id_ex_call<=call;
	id_ex_instr<=if_id_instr;
	id_ex_call_addr_new<=call_addr_new;
	id_ex_p1_addr<= p1_addr;
	id_ex_p0<=p0;
	id_ex_p1<=p1;
	id_ex_p0_addr_new<=p0_addr_new;
	id_ex_p1_addr_new<=p1_addr_new;
	id_ex_dst_addr_new<=dst_addr_new;
	id_ex_sign_extend<=sign_extend;
	id_ex_opcode<=opcode;
	id_ex_ccc<=ccc;
	id_ex_sw_select<=sw_select;
	id_ex_lhb<=lhb;
	id_ex_llb<=llb;
	id_ex_shift<=shift;
	id_ex_mem<=mem;
	id_ex_mem_reg<=mem_reg;
	id_ex_we_dm<=we_dm;
	id_ex_re_dm<=re_dm;
	id_ex_we<=we;
	id_ex_return<=return_addr;
	id_ex_hlt<=hlt;
	id_ex_branch_address<=branch_address;
	id_ex_lw<=lw;
	id_ex_ret<=ret;
	id_ex_b_cond<=b_cond;
	end
	
end

////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////STALL DETECTION UNIT//////////////////////////
/////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

always@(*)
begin
		//If there is a load word data dependency, pipeline is stalled for a cycle to aid memory forwarding
		id_raw_stall=0;		
		if(lw||lhb||ret)
			begin
			if(id_ex_dst_addr_new==p0_addr_new && id_ex_dst_addr_new!=0 && id_ex_lw) 
			id_raw_stall=1;
			end
		else if(b_cond || call || hlt)
			id_raw_stall=0;  
		else  
			begin
			if( (id_ex_dst_addr_new==p0_addr_new || id_ex_dst_addr_new==p1_addr_new) && id_ex_dst_addr_new!=0 && id_ex_lw)
				id_raw_stall=1;
			end     	
end

assign stall= id_raw_stall;


/////////////////////////////////////////////////////////////////////////////
////////////////////////////END OF STALL DETECT/////////////////////////////
///////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////
/////////////////////////////Branch logic///////////////////////
////////////////////////////////////////////////////////////////

//Control lines- branch, branch_cond
always@(*)
branch_cond=(ex_mem_opcode==4'b1100)?1:0;

always@(*)
begin
  branch=0;
  if(branch_cond)
  begin
	case(ex_mem_ccc)


		NE: begin
			if(!store_z)
			branch=1;
			end
		E:  begin
			if(store_z)
			branch=1;
			end
		GT:  begin
			if( !store_z && !store_n)
			branch=1;
			end
		LT:  begin
			if(store_n)
			branch=1;
			end
		GTE:  begin
			if( store_z || (!store_z && !store_n))
			branch=1;
			end
		LTE:  begin
			if(store_n || store_z)
			branch=1;
			end
		OV: begin
			if(store_v)
			branch=1;
			end
		UNCOND:  begin
			 branch=1;
			end
		
  	   endcase
	end
end


/////////////////////////////////////////////////////////////////////////
/////////////////////MUX before ALU/////////////////////////////////////
////////////////////////////////////////////////////////////////////////

always@(*)
begin

	
    shift_amt=id_ex_p1_addr;

	if( id_ex_mem)
	p1_mux_2=id_ex_sign_extend;
	else
	p1_mux_2=p1_mux;

end

//////////////////////////////////////////////////////////////////////////////
/////////////////Muxing logic before ALU to aid forwarding///////////////////
////////////////////////////////////////////////////////////////////////////

always@(*)
begin
p0_mux=id_ex_p0;
p1_mux=id_ex_p1;
	
	if(ex_forward && ex_forward_1)
		p0_mux=ex_mem_result;
	else if(mem_forward && mem_forward_1)
		p0_mux=mem_wb_read_mux;

			
	if(ex_forward && ex_forward_2)
		p1_mux=ex_mem_result;
	else if(mem_forward && mem_forward_2)
		p1_mux=mem_wb_read_mux;
	
end
	
//Instantiate ALU
	
wiscf15_alu ALU(.src0(p0_mux),.src1(p1_mux_2),.shft(shift_amt),.func(id_ex_opcode),.result(result),.v(v),.z(z),.n(n));	
	


/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////Forwarding detection logic//////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*If data dependency exists between source registers and destination registers, the appropriate data is forwarded from EX/MEM pipeline regs
 or MEM/WB pipeline regs. In case of load word data dependency, data is forwarded from Mem/Wb pipeline reg. In case of other data dependencies
 the data is forwarded from Ex/Mem pipeline reg*/
always@(*)
begin
	
	ex_forward_1=0;
	ex_forward_2=0;
	
	

	if(id_ex_lw||id_ex_lhb||id_ex_ret)
			begin
			
			if(ex_mem_dst_addr_new==id_ex_p0_addr_new && ex_mem_dst_addr_new!=0 )
			
				 ex_forward_1=1; 
			
			end

	else if(id_ex_b_cond || id_ex_call || id_ex_hlt)
			begin ex_forward_1=0; ex_forward_2=0; end
	
	else  
			begin

			if( (ex_mem_dst_addr_new==id_ex_p0_addr_new) && ex_mem_dst_addr_new!=0 )
				ex_forward_1=1;	
		
			if( ( ex_mem_dst_addr_new==id_ex_p1_addr_new) && ex_mem_dst_addr_new!=0 )
				ex_forward_2=1;
			end


end

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

always@(*)
begin
	
	mem_forward_1=0;
	mem_forward_2=0;

	if(id_ex_lw||id_ex_lhb||id_ex_ret)
			begin
			
			if(mem_wb_dst_addr_new==id_ex_p0_addr_new && mem_wb_dst_addr_new!=0)
			
			mem_forward_1=1;
			
			end

	else if(id_ex_b_cond || id_ex_call || id_ex_hlt)
			mem_forward_1=0; 
	
	else  
			begin

			if( (mem_wb_dst_addr_new==id_ex_p0_addr_new ) && mem_wb_dst_addr_new!=0)
				 mem_forward_1=1;   
		
			if ( ( mem_wb_dst_addr_new==id_ex_p1_addr_new ) && mem_wb_dst_addr_new!=0)
				mem_forward_2=1;
			end

end

assign mem_forward= mem_forward_1 || mem_forward_2;
assign ex_forward= ex_forward_1 || ex_forward_2;

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////End of forwarding detection logic////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
/////////////////Flopping v,z,n flags////////////////////////////////
////////////////////////////////////////////////////////////////////

always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
	store_v<=0;
	store_z<=0;
	store_n<=0;
	end

	else if(!ex_mem_reg_disable)
	begin
	if(id_ex_opcode[3]==0 && id_ex_opcode!=4'b0001 && id_ex_instr!=16'h0000)
	begin
	store_v<=v;
	store_z<=z;
	store_n<=n;
	end
	end
end

assign lhb_result={id_ex_instr[7:0],p0_mux[7:0]};
assign llb_result={{8{id_ex_instr[7]}},id_ex_instr[7:0]};

//Logic to choose the correct result
always@(*)
begin

	if(id_ex_lhb)	
	result_muxed= lhb_result;
	else if(id_ex_llb)
	result_muxed= llb_result;
	else 
	result_muxed=result;
end
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////EX/MEM Pipeline registers//////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

always@(posedge clk or negedge rst_n)
begin

	if(!rst_n)
	begin
	ex_mem_instr<=0;
	ex_mem_opcode<=0;
	ex_mem_p0_addr_new<=0;
	ex_mem_p1_addr_new<=0;
	ex_mem_dst_addr_new<=0;
	ex_mem_ccc<=0;
	ex_mem_v<=0;
	ex_mem_z<=0;
	ex_mem_n<=0;
	ex_mem_result<=0;
	ex_mem_p0<=0;
	ex_mem_mem<=0;
	ex_mem_mem_reg<=0;
	ex_mem_we_dm<=0;
	ex_mem_re_dm<=0;
	ex_mem_we<=0;
	ex_mem_lhb<=0;
	ex_mem_llb<=0;
	ex_mem_lhb_result<=0;
	ex_mem_llb_result<=0;
	ex_mem_p1<=0;
	ex_mem_return<=0;
	ex_mem_call<=0;
	ex_mem_hlt<=0;
	ex_mem_dst_addr<=0;
	ex_mem_p1_mux<=0;
	end

	else if(!ex_mem_reg_disable && ex_mem_flush)
	begin
	ex_mem_instr<=0;
	ex_mem_opcode<=0;
	ex_mem_p0_addr_new<=0;
	ex_mem_p1_addr_new<=0;
	ex_mem_dst_addr_new<=0;
	ex_mem_ccc<=0;
	ex_mem_v<=0;
	ex_mem_z<=0;
	ex_mem_n<=0;
	ex_mem_result<=0;
	ex_mem_p0<=0;
	ex_mem_mem<=0;
	ex_mem_mem_reg<=0;
	ex_mem_we_dm<=0;
	ex_mem_re_dm<=0;
	ex_mem_we<=0;
	ex_mem_lhb<=0;
	ex_mem_llb<=0;
	ex_mem_lhb_result<=0;
	ex_mem_llb_result<=0;
	ex_mem_p1<=0;
	ex_mem_return<=0;
	ex_mem_call<=0;
	ex_mem_hlt<=0;
	ex_mem_dst_addr<=0;
	ex_mem_p1_mux<=0;
	end

	else if(!ex_mem_reg_disable)
	begin
	
	ex_mem_instr<=id_ex_instr;
	ex_mem_opcode<=id_ex_opcode;
	ex_mem_ccc<=id_ex_ccc;
	ex_mem_call<=id_ex_call;
	ex_mem_call_addr_new<=id_ex_call_addr_new;
	ex_mem_p0_addr_new<=id_ex_p0_addr_new;
	ex_mem_p1_addr_new<=id_ex_p1_addr_new;
	ex_mem_dst_addr_new<=id_ex_dst_addr_new;
	ex_mem_v<=v;
	ex_mem_z<=z;
	ex_mem_n<=n;
	ex_mem_result<=result_muxed;
	ex_mem_p0<=id_ex_p0;
	ex_mem_mem<=id_ex_mem;
	ex_mem_mem_reg<=id_ex_mem_reg;
	ex_mem_we_dm<=id_ex_we_dm;
	ex_mem_re_dm<=id_ex_re_dm;
	ex_mem_we<=id_ex_we;
	ex_mem_lhb<=id_ex_lhb;
	ex_mem_llb<=id_ex_llb;
	ex_mem_lhb_result<=lhb_result;
	ex_mem_llb_result<=llb_result;
	ex_mem_p1<=id_ex_p1;
	ex_mem_return<=id_ex_return;
	ex_mem_hlt<=id_ex_hlt;
	ex_mem_dst_addr<=id_ex_dst_addr;
	ex_mem_branch_address<=id_ex_branch_address;
	ex_mem_p1_mux<=p1_mux;
	end
	
end
	
//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////DCACHE AND UNIFIED MEMORY/////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////



assign addr_to_dcache=ex_mem_result[15:2];
assign offset_to_dcache=ex_mem_result[1:0];

assign dCache_we=(dCache_hit)?ex_mem_we_dm:dm_we; //Write enable is set by state machine in case of dCache miss. On dCache hit, we depends on ex/mem we signal.
assign dCache_re=1'b1;

assign dCache_wdirty=(dCache_hit && ex_mem_we_dm)?1:0; //Dirty bit is set if there is a hit and we write into the data cache.

always_comb
begin
  if(offset_to_dcache==2'b00)
     wrt_dcache_from_pipeline= { read_data_from_dcache[63:16], ex_mem_p1_mux };
  else if(offset_to_dcache==2'b01)
     wrt_dcache_from_pipeline= { read_data_from_dcache[63:32], ex_mem_p1_mux, read_data_from_dcache[15:0] };  
  else if(offset_to_dcache==2'b10)
     wrt_dcache_from_pipeline= { read_data_from_dcache[63:48], ex_mem_p1_mux, read_data_from_dcache[31:0] }; 
  else
     wrt_dcache_from_pipeline= {  ex_mem_p1_mux, read_data_from_dcache[47:0] };
end
	
always_comb
begin
  if(offset_to_dcache==2'b00)
     read_data= read_data_from_dcache[15:0];
  else if(offset_to_dcache==2'b01)
     read_data=read_data_from_dcache[31:16];  
  else if(offset_to_dcache==2'b10)
     read_data=read_data_from_dcache[47:32]; 
  else
     read_data=read_data_from_dcache[63:48];
end

//Mux to select the data input to data cache
always_comb
  if(select_pipeline)
    wrt_dcache=wrt_dcache_from_pipeline;
  else
    wrt_dcache=wrt_dcache_from_unified_mem;

//Mux to select the address to unified memory
always_comb
  if(select_data_addr)
    addr_muxed=addr_to_dcache;
  else
    addr_muxed=pc_to_cache;

cache dataCache (.clk(clk),.rst_n(rst_n),.addr(addr_to_dcache),.wr_data(wrt_dcache),.wdirty(dCache_wdirty),.we(dCache_we),.re(dCache_re),
							.rd_data(read_data_from_dcache),.tag_out(dCache_tag_out),.hit(dCache_hit),.dirty(dCache_dirty));

unified_mem uniMemory(.clk(clk),.rst_n(rst_n),.addr(addr_muxed),.re(um_re),
							.we(um_we),.wdata(read_data_from_dcache),.rd_data(wrt_dcache_from_unified_mem),.rdy(rdy));

//Mux to choose between ALU result and read data from data cache							
always@(*)
begin
	if(ex_mem_re_dm)
		read_mux=read_data;
	else	
		read_mux=ex_mem_result;
end

////////////////////////////////////////////////////////////////
///////////////MEM/WB Pipeline registers///////////////////////
//////////////////////////////////////////////////////////////

always@(posedge clk or negedge rst_n)
begin

	if(!rst_n)
	begin
	mem_wb_read_mux<=0;
	mem_wb_instr<=0;
	mem_wb_read_data<=0;
	mem_wb_mem_reg<=0;
	mem_wb_p0_addr_new<=0;
	mem_wb_p1_addr_new<=0;
	mem_wb_dst_addr_new<=0;
	mem_wb_p0<=0;
	mem_wb_lhb<=0;
	mem_wb_llb<=0;
	mem_wb_call<=0;
	mem_wb_result<=0;
	mem_wb_return<=0;
	mem_wb_hlt<=0;
	mem_wb_we<=0;
	mem_wb_dst_addr<=0;
	mem_wb_lhb_result<=0;
	mem_wb_llb_result<=0;
	mem_wb_opcode<=0;
	mem_wb_v<=0;
	mem_wb_z<=0;
	mem_wb_n<=0;
	end
    else if(!mem_wb_reg_disable && mem_wb_flush)
	begin
	mem_wb_read_mux<=0;
	mem_wb_instr<=0;
	mem_wb_read_data<=0;
	mem_wb_mem_reg<=0;
	mem_wb_p0_addr_new<=0;
	mem_wb_p1_addr_new<=0;
	mem_wb_dst_addr_new<=0;
	mem_wb_p0<=0;
	mem_wb_lhb<=0;
	mem_wb_llb<=0;
	mem_wb_lhb_result<=0;
	mem_wb_llb_result<=0;
	mem_wb_call<=0;
	mem_wb_result<=0;
	mem_wb_return<=0;
	mem_wb_hlt<=0;
	mem_wb_we<=0;
	mem_wb_dst_addr<=0;
	mem_wb_opcode<=0;
	mem_wb_v<=0;
	mem_wb_z<=0;
	mem_wb_n<=0;
	end

	else if(!mem_wb_reg_disable)
	begin
	mem_wb_read_mux<=read_mux;
	mem_wb_instr<=ex_mem_instr;
	mem_wb_opcode<=ex_mem_opcode;
	mem_wb_call_addr_new<=ex_mem_call_addr_new;
	mem_wb_read_data<=read_data;
	mem_wb_mem_reg<=ex_mem_mem_reg;
	mem_wb_p0_addr_new<=ex_mem_p0_addr_new;
	mem_wb_p1_addr_new<=ex_mem_p1_addr_new;
	mem_wb_dst_addr_new<=ex_mem_dst_addr_new;
	mem_wb_p0<=ex_mem_p0;
	mem_wb_lhb<=ex_mem_lhb;
	mem_wb_llb<=ex_mem_llb;
	mem_wb_lhb_result<=ex_mem_lhb_result;
	mem_wb_llb_result<=ex_mem_llb_result;
	mem_wb_call<=ex_mem_call;
	mem_wb_result<=ex_mem_result;
	mem_wb_call_addr_new<=ex_mem_call_addr_new;
	mem_wb_return<=ex_mem_return;
	mem_wb_hlt<=ex_mem_hlt;
	mem_wb_we<=ex_mem_we;
	mem_wb_dst_addr<=ex_mem_dst_addr;
	mem_wb_v<=ex_mem_v;
	mem_wb_z<=ex_mem_z;
	mem_wb_n<=ex_mem_n;
	end
end


////////////////////////////////////////////////////////////////////////////////////////
///////////////////MUX to choose register file write input/////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

always@(*)
begin
	if(mem_wb_mem_reg)
	dst=mem_wb_read_data;
	else if(mem_wb_call)
	dst=return_addr;
	else if(mem_wb_lhb)
	dst=mem_wb_lhb_result;
    else if(mem_wb_llb)
	dst=mem_wb_llb_result;
	else
	dst=mem_wb_result;

end

endmodule

//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////END OF CODE///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
