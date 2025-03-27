module program_counter (
    input clk,                // Clock signal
    input reset,              // Reset signal (active high)
    input [15:0] pc_next,     // Next PC value (address of the next instruction)
    output reg [15:0] pc_out  // Current PC value (address of the current instruction)
);
    // Initialize the PC to 0 on reset
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 16'd0; // Reset the PC to 0
        end
        else begin
            pc_out <= pc_next; // Update the PC to the next address
        end
    end
endmodule

module instr_mem(
    input [15:0] pc,
    output [15:0] instruction
);
    reg [15:0] memory [0:255];
    initial begin
        memory[0] = 16'b000_001_010_011_0000; // add $3, $1, $2
        memory[1] = 16'b000_001_010_011_0001; // sub $3, $1, $2
        memory[2] = 16'b000_001_010_011_0010; // and $3, $1, $2
        memory[3] = 16'b000_001_010_011_0011; // or $3, $1, $2
        memory[4] = 16'b000_001_010_011_0100; // slt $3, $1, $2
        memory[5] = 16'b000_111_000_000_1000; // jr memory[6]
        memory[6] = 16'b100_010_011_0000111;  // lw $3, 7($2)
        memory[7] = 16'b101_010_011_0000111;  // sw $3, 7($2)
        memory[8] = 16'b110_001_010_0000000;  // beq $4, $5, 0
        memory[9] = 16'b111_010_001_0000111;  // addi $1, $2, 7
        memory[10] = 16'b010_0000000001011;       // j to address 22
        memory[11] = 16'b011_0000000001100;   // jal address 24
        memory[12] = 16'b001_010_001_0000111; // slti $1, $2, 7
        // Fill the rest of memory with NOP
        for (integer i = 13; i < 256; i = i + 1) begin
            memory[i] = 16'b0000000000000000; // NOP
        end
    end
    assign instruction = memory[pc >> 1]; // Word-addressed memory
endmodule

module controlunit(
    input [2:0] opcode,
    input reset,
    output reg[1:0] aluop,  
    output reg jump,branch,memread,memwrite,alusrc,regwrite,regdst,memtoreg
);

always @(*) begin
    if(reset) begin
        regdst=2'b00;
        memtoreg=2'b00;
        aluop=2'b00;
        jump=1'b0;
        branch=1'b0;
        memread=1'b0;
        memwrite=1'b0;
        alusrc=1'b0;
        regwrite=1'b0;
    end
    else begin
        case(opcode)
            3'b000: begin // R-type 
                    regdst = 1'b1;  
                    memtoreg = 1'b0;  
                    aluop = 2'b00;  
                    jump = 1'b0;  
                    branch = 1'b0;  
                    memread = 1'b0;  
                    memwrite = 1'b0;  
                    alusrc = 1'b0;  
                    regwrite = 1'b1;  
                
                    end  
            3'b001: begin // stil 
                    regdst = 1'b0;  
                    memtoreg = 1'b0;  
                    aluop = 2'b10;  
                    jump = 1'b0;  
                    branch = 1'b0;  
                    memread = 1'b0;  
                    memwrite = 1'b0;  
                    alusrc = 1'b1;  
                    regwrite = 1'b1;  
                  
                    end  
            3'b010: begin // j  
                    regdst = 1'b0;  
                    memtoreg = 1'b0;  
                    aluop = 2'b00;  
                    jump = 1'b1;  
                    branch = 1'b0;  
                    memread = 1'b0;  
                    memwrite = 1'b0;  
                    alusrc = 1'b0;  
                    regwrite = 1'b0;  
  
                    end  
            3'b011: begin // jal  
                    regdst = 1'b1;  
                    memtoreg = 1'b1;  
                    aluop = 2'b00;  
                    jump = 1'b1;  
                    branch = 1'b0;  
                    memread = 1'b0;  
                    memwrite = 1'b0;  
                    alusrc = 1'b0;  
                    regwrite = 1'b1;  
  
                    end  
            3'b100: begin // lw  
                    regdst = 1'b0;  
                    memtoreg = 1'b1;  
                    aluop = 2'b11;  
                    jump = 1'b0;  
                    branch = 1'b0;  
                    memread = 1'b1;  
                    memwrite = 1'b0;  
                    alusrc = 1'b1;  
                    regwrite = 1'b1;    
                    end  
            3'b101: begin // sw  
                    regdst = 1'b0;  
                    memtoreg = 1'b0;  
                    aluop = 2'b11;  
                    jump = 1'b0;  
                    branch = 1'b0;  
                    memread = 1'b0;  
                    memwrite = 1'b1;  
                    alusrc = 1'b1;  
                    regwrite = 1'b0;  
 
                    end  
            3'b110: begin // beq  
                    regdst = 1'b0;  
                    memtoreg = 1'b0;  
                    aluop = 2'b01;  
                    jump = 1'b0;  
                    branch = 1'b1;  
                    memread = 1'b0;  
                    memwrite = 1'b0;  
                    alusrc = 1'b0;  
                    regwrite = 1'b0;  
 
                    end  
            3'b111: begin // addi  
                    regdst = 2'b00;  
                    memtoreg = 2'b00;  
                    aluop = 2'b11;  
                    jump = 1'b0;  
                    branch = 1'b0;  
                    memread = 1'b0;  
                    memwrite = 1'b0;  
                    alusrc = 1'b1;  
                    regwrite = 1'b1;  

                    end  
            default: begin  
                    regdst = 2'b01;  
                    memtoreg = 2'b00;  
                    aluop = 2'b00;  
                    jump = 1'b0;  
                    branch = 1'b0;  
                    memread = 1'b0;  
                    memwrite = 1'b0;  
                    alusrc = 1'b0;  
                    regwrite = 1'b1;  
                    end  
        endcase
    end
end

endmodule


module alucontrolunit(
    input [1:0] aluop,
    input [3:0] functioncode,
    output reg [2:0] aluop_out
);
    always @(*) begin
        case(aluop)
            2'b00: begin  // R-type instructions
                case(functioncode)
                    4'b0000: aluop_out = 3'b000;  // add
                    4'b0001: aluop_out = 3'b001;  // sub
                    4'b0010: aluop_out = 3'b010;  // and
                    4'b0011: aluop_out = 3'b011;  // or
                    4'b0100: aluop_out = 3'b100;  // slt
                    default: aluop_out = 3'b000;  // default to add
                endcase
            end
            2'b01: aluop_out = 3'b001;  // beq (subtract)
            2'b10: aluop_out = 3'b100;  // slti
            2'b11: aluop_out = 3'b000;  // lw/sw/addi (add)
            default: aluop_out = 3'b000; // default to add
        endcase
    end
endmodule

module JRcontrol( input[1:0] alu_op, 
       input [3:0] functioncode,
       output JRcontrol
    );
assign JRcontrol = ({alu_op,functioncode}==6'b001000) ? 1'b1 : 1'b0;
endmodule

module register(
    input [2:0] readregister1,
    input [2:0] readregister2,
    input [2:0] writeregister,
    input [15:0] writedata,
    input regwrite,
    output [15:0] readdata1,
    output [15:0] readdata2,
    input clk,
    input reset
);

reg [15:0] reg_array [7:0]; 

integer i;


always @(posedge clk or posedge reset) begin
    if (reset) begin
        reg_array[0] <= 16'b0;
        reg_array[1] <= 16'd2;
        reg_array[2] <= 16'd1;
        reg_array[3] <= 16'b0;
        reg_array[4] <= 16'd1;
        reg_array[5] <= 16'd1;
        reg_array[6] <= 16'b0;
        reg_array[7] <= 16'd12;
    end 
end
always @(negedge clk) begin
    if (regwrite) begin
        reg_array[writeregister] <= writedata;
    end
end


assign readdata1 = reg_array[readregister1];
assign readdata2 = reg_array[readregister2];

endmodule

module alu(
    input [15:0] data1,
    input [15:0] data2,
    input [2:0] alu_op,
    output zero,
    output [15:0] alu_result
);
reg [15:0] aluresulttemp;

always @(*)begin
        case(alu_op) 
        3'b000: aluresulttemp= data1+data2;  //r-type add
        3'b001: aluresulttemp= data1-data2;  //r-type sub
        3'b010: aluresulttemp= data1&data2;  //r-type and
        3'b011: aluresulttemp= data1|data2;//r-type or
        3'b100: aluresulttemp = (data1<data2)? 16'd1:16'd0;  //r-type slt
        default: aluresulttemp = data1+data2;//r-type add
        endcase
 end
 assign alu_result =aluresulttemp;
 assign zero = (alu_result == 16'd0) ? 1'b1 : 1'b0;

endmodule

module data_memory (
    input clk, 
    input memwrite, 
    input memread,
    input [15:0] address,
    input [15:0] writedata,
    output reg [15:0] memreaddata
    );

    reg[15:0] mem [0:255];
    
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            mem[i] = 16'b0;
        end
    end


    always @(*) begin
        if (memread) begin
            memreaddata = mem[address];
        end
        else begin
            memreaddata = 16'd0; 
        end
    end

    always @(negedge clk) begin
        if(memwrite)begin
            mem[address]<=writedata;
        end
    end
endmodule

module mux3bit (
 input [2:0] a,
 input [2:0] b,
 input higha,
 output [2:0] out
);

assign out = higha ? a:b;
endmodule

module  mux16bit( 
 input [15:0] a,
 input [15:0] b,
 input higha,
 output [15:0] out
);
assign out = higha ? a:b;
endmodule

module jump_address_constructor(
    input [12:0] instruction, 
    input [15:14] PC_plus_2, 
    output [15:0] out);

    assign out = {PC_plus_2, instruction<<1};
endmodule

module adder (
    input [15:0] a, 
    input [15:0] b, 
    output [15:0] out);
  assign out = a + b;
endmodule

module processor_wire(
    input clk,
    input reset
);

wire [15:0] address;
wire [15:0] pc_adder_mux;
wire  [15:0] instruction;
wire  [1:0] aluop;
wire  jump,branch,memread,memwrite,alusrc,regwrite,regdst,memtoreg;
reg [15:0] two =2;
wire [2:0]writeregister;
wire [15:0]readdata1;
wire [15:0]readdata2;
wire [15:0]writedataR;
wire [2:0] aluop_out;
wire [15:0]aludata2;
wire zero;
wire [15:0]aluresult;
wire [15:0]memdata;
wire [15:0]jumpaddress;
wire [15:0]branchaddress;
wire [15:0]branch_adder_mux;
wire [15:0] jump_mux_address;
wire [15:0] nxt_address;
wire JRcontrol;

program_counter pc( .clk(clk), .reset(reset), .pc_next(nxt_address), .pc_out(address));
adder pc_incremeter( .a(address), .b(two), .out(pc_adder_mux));
jump_address_constructor jumper( .instruction(instruction[12:0]), .PC_plus_2(pc_adder_mux[15:14]), .out(jumpaddress));
mux16bit jump_mux(.a(jumpaddress),.b(branchaddress), .higha(jump), .out(jump_mux_address));
adder branch_add(.a(pc_adder_mux), .b({12'd0, instruction[3:0]} << 1),.out(branch_adder_mux));
mux16bit branch_mux(.a(branch_adder_mux),.b(pc_adder_mux), .higha(branch&zero), .out(branchaddress));

instr_mem imem(.pc(address),.instruction(instruction));
controlunit control(.opcode(instruction[15:13]),.reset(reset), .regdst(regdst), .memtoreg(memtoreg), .aluop(aluop), .jump(jump),
.branch(branch),.memread(memread),.memwrite(memwrite),.alusrc(alusrc),.regwrite(regwrite));
alucontrolunit alucontrol(.aluop(aluop),.functioncode(instruction[3:0]), .aluop_out(aluop_out));

register registers(.readregister1(instruction[12:10]), .readregister2(instruction[9:7]), .writeregister(writeregister),
.writedata(writedataR), .regwrite(regwrite), .clk(clk), .readdata1(readdata1), .readdata2(readdata2), .reset(reset) );
mux3bit writeregister_mux(.a(instruction[6:4]),.b(instruction[9:7]),.higha(regdst),.out(writeregister));

mux16bit writedata_mux(.a(memdata),.b(aluresult),.higha(memtoreg),.out(writedataR));

mux16bit alu_mux(.a({ 12'd0,instruction[3:0]}),.b(readdata2),.higha(alusrc),.out(aludata2));
alu alublock(.data1(readdata1), .data2(aludata2), .alu_op(aluop_out), .zero(zero), .alu_result(aluresult));
data_memory dmem(.clk(clk), .memwrite(memwrite), .memread(memread), .address(aluresult), .writedata(readdata2), .memreaddata(memdata));

JRcontrol JRcontrolnblock( .alu_op(aluop),.functioncode(instruction[3:0]),.JRcontrol(JRcontrol));
mux16bit JR_mux(.a(readdata1), .b(jump_mux_address), .higha(JRcontrol), .out(nxt_address));
endmodule