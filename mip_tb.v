module mip_tb();
reg clk;
reg reset;
wire reg_array;

initial begin
    clk=0; //generate clock
end
always #100 clk=~clk;

processor_wire processor(.clk(clk),.reset(reset));



initial begin
    $dumpfile("mip.vcd");
    $dumpvars(0,mip_tb);
    $monitor("time=%t | register$3=%b| register$2=%b| memory$8=%b | register$1=%b |" ,$time, processor.registers.reg_array[3],processor.registers.reg_array[2], processor.dmem.mem[8],processor.registers.reg_array[1]);
    reset=1;
    #100;
    reset=0;
    #5000;
    $finish;
end

endmodule