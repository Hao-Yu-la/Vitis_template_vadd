
?
Command: %s
53*	vivadotcl2i
Usynth_design -top rtl_kernel_wizard_0 -part xcu280-fsvh2892-2L-e -mode out_of_context2default:defaultZ4-113h px? 
:
Starting synth_design
149*	vivadotclZ4-321h px? 
?
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2default:default2
xcu2802default:defaultZ17-347h px? 
?
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2default:default2
xcu2802default:defaultZ17-349h px? 
[
Loading part %s157*device2(
xcu280-fsvh2892-2L-e2default:defaultZ21-403h px? 
?
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
42default:defaultZ8-7079h px? 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px? 
a
#Helper process launched with PID %s4824*oasys2
1020222default:defaultZ8-7075h px? 
?
%s*synth2?
?Starting RTL Elaboration : Time (s): cpu = 00:00:03 ; elapsed = 00:00:03 . Memory (MB): peak = 3251.035 ; gain = 186.688 ; free physical = 2418 ; free virtual = 54999
2default:defaulth px? 
?
synthesizing module '%s'%s4497*oasys2'
rtl_kernel_wizard_02default:default2
 2default:default2?
|/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0.v2default:default2
72default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys25
!rtl_kernel_wizard_0_control_s_axi2default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_control_s_axi.v2default:default2
62default:default8@Z8-6157h px? 
h
%s
*synth2P
<	Parameter C_S_AXI_ADDR_WIDTH bound to: 12 - type: integer 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter C_S_AXI_DATA_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
?
-case statement is not full and has no default155*oasys2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_control_s_axi.v2default:default2
2072default:default8@Z8-155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys25
!rtl_kernel_wizard_0_control_s_axi2default:default2
 2default:default2
12default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_control_s_axi.v2default:default2
62default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2/
rtl_kernel_wizard_0_example2default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example.sv2default:default2
52default:default8@Z8-6157h px? 
j
%s
*synth2R
>	Parameter C_M00_AXI_ADDR_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
k
%s
*synth2S
?	Parameter C_M00_AXI_DATA_WIDTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys24
 rtl_kernel_wizard_0_example_vadd2default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_vadd.sv2default:default2
62default:default8@Z8-6157h px? 
h
%s
*synth2P
<	Parameter C_M_AXI_ADDR_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
i
%s
*synth2Q
=	Parameter C_M_AXI_DATA_WIDTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter C_XFER_SIZE_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter C_ADDER_BIT_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2?
+rtl_kernel_wizard_0_example_axi_read_master2default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_axi_read_master.sv2default:default2
522default:default8@Z8-6157h px? 
h
%s
*synth2P
<	Parameter C_M_AXI_ADDR_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
i
%s
*synth2Q
=	Parameter C_M_AXI_DATA_WIDTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter C_XFER_SIZE_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter C_MAX_OUTSTANDING bound to: 8 - type: integer 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter C_INCLUDE_DATA_FIFO bound to: 1 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2!
xpm_fifo_sync2default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
19512default:default8@Z8-6157h px? 
g
%s
*synth2O
;	Parameter FIFO_MEMORY_TYPE bound to: auto - type: string 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter ECC_MODE bound to: no_ecc - type: string 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter FIFO_WRITE_DEPTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter WRITE_DATA_WIDTH bound to: 513 - type: integer 
2default:defaulth p
x
? 
i
%s
*synth2Q
=	Parameter WR_DATA_COUNT_WIDTH bound to: 10 - type: integer 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter PROG_FULL_THRESH bound to: 10 - type: integer 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter FULL_RESET_VALUE bound to: 1 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter USE_ADV_FEATURES bound to: 1F1F - type: string 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter READ_MODE bound to: fwft - type: string 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter FIFO_READ_LATENCY bound to: 2 - type: integer 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter READ_DATA_WIDTH bound to: 513 - type: integer 
2default:defaulth p
x
? 
i
%s
*synth2Q
=	Parameter RD_DATA_COUNT_WIDTH bound to: 10 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter PROG_EMPTY_THRESH bound to: 10 - type: integer 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter DOUT_RESET_VALUE bound to: 0 - type: string 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter WAKEUP_TIME bound to: 0 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2!
xpm_fifo_base2default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
562default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2$
xpm_counter_updn2default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2$
xpm_counter_updn2default:default2
 2default:default2
22default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys24
 xpm_counter_updn__parameterized02default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 xpm_counter_updn__parameterized02default:default2
 2default:default2
22default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2#
xpm_memory_base2default:default2
 2default:default2Z
D/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv2default:default2
572default:default8@Z8-6157h px? 
?
Synth Info: %s 4384*oasys2?
?[XPM_MEMORY 20-1] MEMORY_PRIMITIVE (0) instructs Vivado Synthesis to choose the memory primitive type. Depending on their values, other XPM_MEMORY parameters may preclude the choice of certain memory primitive types. Review XPM_MEMORY documentation and parameter values to understand any limitations, or set MEMORY_PRIMITIVE to a different value. 2default:default2Z
D/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv2default:default2
4902default:default8@Z8-6059h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2#
xpm_memory_base2default:default2
 2default:default2
32default:default2
12default:default2Z
D/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv2default:default2
572default:default8@Z8-6155h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12062default:default8@Z8-226h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12732default:default8@Z8-226h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12952default:default8@Z8-226h px? 
?
synthesizing module '%s'%s4497*oasys2$
xpm_fifo_reg_bit2default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
19062default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2$
xpm_fifo_reg_bit2default:default2
 2default:default2
42default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
19062default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys24
 xpm_counter_updn__parameterized12default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 xpm_counter_updn__parameterized12default:default2
 2default:default2
42default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2 
xpm_fifo_rst2default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
16182default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2 
xpm_fifo_rst2default:default2
 2default:default2
52default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
16182default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys24
 xpm_counter_updn__parameterized22default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 xpm_counter_updn__parameterized22default:default2
 2default:default2
52default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys24
 xpm_counter_updn__parameterized32default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 xpm_counter_updn__parameterized32default:default2
 2default:default2
52default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
xpm_fifo_base2default:default2
 2default:default2
62default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
562default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
xpm_fifo_sync2default:default2
 2default:default2
72default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
19512default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys27
#rtl_kernel_wizard_0_example_counter2default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6157h px? 
]
%s
*synth2E
1	Parameter C_WIDTH bound to: 20 - type: integer 
2default:defaulth p
x
? 
b
%s
*synth2J
6	Parameter C_INIT bound to: 20'b00000000000000000000 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys27
#rtl_kernel_wizard_0_example_counter2default:default2
 2default:default2
82default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2G
3rtl_kernel_wizard_0_example_counter__parameterized02default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6157h px? 
\
%s
*synth2D
0	Parameter C_WIDTH bound to: 4 - type: integer 
2default:defaulth p
x
? 
Q
%s
*synth29
%	Parameter C_INIT bound to: 4'b1000 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2G
3rtl_kernel_wizard_0_example_counter__parameterized02default:default2
 2default:default2
82default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2?
+rtl_kernel_wizard_0_example_axi_read_master2default:default2
 2default:default2
92default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_axi_read_master.sv2default:default2
522default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys25
!rtl_kernel_wizard_0_example_adder2default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_adder.v2default:default2
112default:default8@Z8-6157h px? 
i
%s
*synth2Q
=	Parameter C_AXIS_TDATA_WIDTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter C_ADDER_BIT_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter C_NUM_CLOCKS bound to: 1 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2!
xpm_fifo_axis2default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
23432default:default8@Z8-6157h px? 
l
%s
*synth2T
@	Parameter CLOCKING_MODE bound to: common_clock - type: string 
2default:defaulth p
x
? 
n
%s
*synth2V
B	Parameter FIFO_MEMORY_TYPE bound to: distributed - type: string 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter PACKET_FIFO bound to: false - type: string 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter FIFO_DEPTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
b
%s
*synth2J
6	Parameter TDATA_WIDTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter TID_WIDTH bound to: 1 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter TDEST_WIDTH bound to: 1 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter TUSER_WIDTH bound to: 1 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter ECC_MODE bound to: no_ecc - type: string 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter RELATED_CLOCKS bound to: 0 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter USE_ADV_FEATURES bound to: 1002 - type: string 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter WR_DATA_COUNT_WIDTH bound to: 6 - type: integer 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter RD_DATA_COUNT_WIDTH bound to: 6 - type: integer 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter PROG_FULL_THRESH bound to: 27 - type: integer 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter PROG_EMPTY_THRESH bound to: 5 - type: integer 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter CDC_SYNC_STAGES bound to: 2 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2$
xpm_cdc_sync_rst2default:default2
 2default:default2T
>/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv2default:default2
10592default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2$
xpm_cdc_sync_rst2default:default2
 2default:default2
102default:default2
12default:default2T
>/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv2default:default2
10592default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys21
xpm_fifo_base__parameterized02default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
562default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys23
xpm_memory_base__parameterized02default:default2
 2default:default2Z
D/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv2default:default2
572default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys23
xpm_memory_base__parameterized02default:default2
 2default:default2
102default:default2
12default:default2Z
D/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv2default:default2
572default:default8@Z8-6155h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12062default:default8@Z8-226h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12732default:default8@Z8-226h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12952default:default8@Z8-226h px? 
?
synthesizing module '%s'%s4497*oasys24
 xpm_counter_updn__parameterized42default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 xpm_counter_updn__parameterized42default:default2
 2default:default2
102default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys24
 xpm_counter_updn__parameterized52default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 xpm_counter_updn__parameterized52default:default2
 2default:default2
102default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys24
 xpm_counter_updn__parameterized62default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 xpm_counter_updn__parameterized62default:default2
 2default:default2
102default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys21
xpm_fifo_base__parameterized02default:default2
 2default:default2
102default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
562default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
xpm_fifo_axis2default:default2
 2default:default2
112default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
23432default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys25
!rtl_kernel_wizard_0_example_adder2default:default2
 2default:default2
122default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_adder.v2default:default2
112default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2@
,rtl_kernel_wizard_0_example_axi_write_master2default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_axi_write_master.sv2default:default2
422default:default8@Z8-6157h px? 
h
%s
*synth2P
<	Parameter C_M_AXI_ADDR_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
i
%s
*synth2Q
=	Parameter C_M_AXI_DATA_WIDTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter C_XFER_SIZE_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter C_MAX_OUTSTANDING bound to: 32 - type: integer 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter C_INCLUDE_DATA_FIFO bound to: 1 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys21
xpm_fifo_sync__parameterized02default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
19512default:default8@Z8-6157h px? 
n
%s
*synth2V
B	Parameter FIFO_MEMORY_TYPE bound to: distributed - type: string 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter ECC_MODE bound to: no_ecc - type: string 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter FIFO_WRITE_DEPTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter WRITE_DATA_WIDTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter WR_DATA_COUNT_WIDTH bound to: 6 - type: integer 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter PROG_FULL_THRESH bound to: 10 - type: integer 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter FULL_RESET_VALUE bound to: 1 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter USE_ADV_FEATURES bound to: 1F1F - type: string 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter READ_MODE bound to: fwft - type: string 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter FIFO_READ_LATENCY bound to: 1 - type: integer 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter READ_DATA_WIDTH bound to: 512 - type: integer 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter RD_DATA_COUNT_WIDTH bound to: 6 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter PROG_EMPTY_THRESH bound to: 10 - type: integer 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter DOUT_RESET_VALUE bound to: 0 - type: string 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter WAKEUP_TIME bound to: 0 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys21
xpm_fifo_base__parameterized12default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
562default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys24
 xpm_counter_updn__parameterized72default:default2
 2default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 xpm_counter_updn__parameterized72default:default2
 2default:default2
122default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
18582default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys23
xpm_memory_base__parameterized12default:default2
 2default:default2Z
D/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv2default:default2
572default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys23
xpm_memory_base__parameterized12default:default2
 2default:default2
122default:default2
12default:default2Z
D/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv2default:default2
572default:default8@Z8-6155h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12062default:default8@Z8-226h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12732default:default8@Z8-226h px? 
?
default block is never used226*oasys2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
12952default:default8@Z8-226h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys21
xpm_fifo_base__parameterized12default:default2
 2default:default2
122default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
562default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys21
xpm_fifo_sync__parameterized02default:default2
 2default:default2
122default:default2
12default:default2V
@/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv2default:default2
19512default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2G
3rtl_kernel_wizard_0_example_counter__parameterized12default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6157h px? 
\
%s
*synth2D
0	Parameter C_WIDTH bound to: 6 - type: integer 
2default:defaulth p
x
? 
S
%s
*synth2;
'	Parameter C_INIT bound to: 6'b111111 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2G
3rtl_kernel_wizard_0_example_counter__parameterized12default:default2
 2default:default2
122default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2G
3rtl_kernel_wizard_0_example_counter__parameterized22default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6157h px? 
\
%s
*synth2D
0	Parameter C_WIDTH bound to: 8 - type: integer 
2default:defaulth p
x
? 
U
%s
*synth2=
)	Parameter C_INIT bound to: 8'b00000000 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2G
3rtl_kernel_wizard_0_example_counter__parameterized22default:default2
 2default:default2
122default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2G
3rtl_kernel_wizard_0_example_counter__parameterized32default:default2
 2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6157h px? 
\
%s
*synth2D
0	Parameter C_WIDTH bound to: 6 - type: integer 
2default:defaulth p
x
? 
S
%s
*synth2;
'	Parameter C_INIT bound to: 6'b100000 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2G
3rtl_kernel_wizard_0_example_counter__parameterized32default:default2
 2default:default2
122default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_counter.sv2default:default2
72default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2@
,rtl_kernel_wizard_0_example_axi_write_master2default:default2
 2default:default2
132default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_axi_write_master.sv2default:default2
422default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys24
 rtl_kernel_wizard_0_example_vadd2default:default2
 2default:default2
142default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example_vadd.sv2default:default2
62default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2/
rtl_kernel_wizard_0_example2default:default2
 2default:default2
152default:default2
12default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_example.sv2default:default2
52default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2'
rtl_kernel_wizard_02default:default2
 2default:default2
162default:default2
12default:default2?
|/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0.v2default:default2
72default:default8@Z8-6155h px? 
?
%s*synth2?
?Finished RTL Elaboration : Time (s): cpu = 00:00:05 ; elapsed = 00:00:05 . Memory (MB): peak = 3312.004 ; gain = 247.656 ; free physical = 1544 ; free virtual = 54127
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:06 ; elapsed = 00:00:06 . Memory (MB): peak = 3329.816 ; gain = 265.469 ; free physical = 2390 ; free virtual = 54972
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:06 ; elapsed = 00:00:06 . Memory (MB): peak = 3329.816 ; gain = 265.469 ; free physical = 2390 ; free virtual = 54972
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.102default:default2
00:00:00.102default:default2
3329.8162default:default2
0.0002default:default2
23682default:default2
549502default:defaultZ17-722h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
>

Processing XDC Constraints
244*projectZ1-262h px? 
=
Initializing timing engine
348*projectZ1-569h px? 
?
Parsing XDC File [%s]
179*designutils2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_ooc.xdc2default:default8Z20-179h px? 
?
Finished Parsing XDC File [%s]
178*designutils2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_ooc.xdc2default:default8Z20-178h px? 
?
Parsing XDC File [%s]
179*designutils2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_user.xdc2default:default8Z20-179h px? 
?
Finished Parsing XDC File [%s]
178*designutils2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/imports/rtl_kernel_wizard_0_user.xdc2default:default8Z20-178h px? 
?
?Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2^
J/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/tcl/xpm_fifo_rst_axis.tcl2default:default29
%.Xil/rtl_kernel_wizard_0_propImpl.xdc2default:defaultZ1-236h px? 
?
?Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2Y
E/tools/Xilinx/Vivado/2021.1/data/ip/xpm/xpm_fifo/tcl/xpm_fifo_rst.tcl2default:default29
%.Xil/rtl_kernel_wizard_0_propImpl.xdc2default:defaultZ1-236h px? 
l
2%s XPM XDC files have been applied to the design.
665*project2
42default:defaultZ1-1714h px? 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
3547.1292default:default2
0.0002default:default2
22342default:default2
548162default:defaultZ17-722h px? 
~
!Unisim Transformation Summary:
%s111*project29
%No Unisim elements were transformed.
2default:defaultZ1-111h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common24
 Constraint Validation Runtime : 2default:default2
00:00:00.292default:default2
00:00:00.122default:default2
3551.0982default:default2
3.9692default:default2
22342default:default2
548162default:defaultZ17-722h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Constraint Validation : Time (s): cpu = 00:00:18 ; elapsed = 00:00:18 . Memory (MB): peak = 3551.098 ; gain = 486.750 ; free physical = 2365 ; free virtual = 54948
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
V
%s
*synth2>
*Start Loading Part and Timing Information
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Loading part: xcu280-fsvh2892-2L-e
2default:defaulth p
x
? 
B
 Reading net delay rules and data4578*oasysZ8-6742h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Loading Part and Timing Information : Time (s): cpu = 00:00:18 ; elapsed = 00:00:18 . Memory (MB): peak = 3551.098 ; gain = 486.750 ; free physical = 2365 ; free virtual = 54948
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Z
%s
*synth2B
.Start Applying 'set_property' XDC Constraints
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:18 ; elapsed = 00:00:18 . Memory (MB): peak = 3551.098 ; gain = 486.750 ; free physical = 2365 ; free virtual = 54948
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2

wstate_reg2default:default25
!rtl_kernel_wizard_0_control_s_axi2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2

rstate_reg2default:default25
!rtl_kernel_wizard_0_control_s_axi2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys20
gen_fwft.curr_fwft_state_reg2default:default2!
xpm_fifo_base2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys20
gen_fwft.curr_fwft_state_reg2default:default21
xpm_fifo_base__parameterized02default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys20
gen_fwft.curr_fwft_state_reg2default:default21
xpm_fifo_base__parameterized12default:defaultZ8-802h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_                  iSTATE |                             0001 |                               11
2default:defaulth p
x
? 
.
%s
*synth2
*
2default:defaulth p
x
? 
?
%s
*synth2s
_                  WRIDLE |                             0010 |                               00
2default:defaulth p
x
? 
?
%s
*synth2s
_                  WRDATA |                             0100 |                               01
2default:defaulth p
x
? 
?
%s
*synth2s
_                  WRRESP |                             1000 |                               10
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2

wstate_reg2default:default2
one-hot2default:default25
!rtl_kernel_wizard_0_control_s_axi2default:defaultZ8-3354h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_                  iSTATE |                              001 |                               10
2default:defaulth p
x
? 
.
%s
*synth2
*
2default:defaulth p
x
? 
?
%s
*synth2s
_                  RDIDLE |                              010 |                               00
2default:defaulth p
x
? 
?
%s
*synth2s
_                  RDDATA |                              100 |                               01
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2

rstate_reg2default:default2
one-hot2default:default25
!rtl_kernel_wizard_0_control_s_axi2default:defaultZ8-3354h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_                 invalid |                               00 |                               00
2default:defaulth p
x
? 
?
%s
*synth2s
_            stage1_valid |                               01 |                               10
2default:defaulth p
x
? 
?
%s
*synth2s
_       both_stages_valid |                               10 |                               11
2default:defaulth p
x
? 
?
%s
*synth2s
_            stage2_valid |                               11 |                               01
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys20
gen_fwft.curr_fwft_state_reg2default:default2

sequential2default:default2!
xpm_fifo_base2default:defaultZ8-3354h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_                 invalid |                               00 |                               00
2default:defaulth p
x
? 
?
%s
*synth2s
_            stage1_valid |                               01 |                               10
2default:defaulth p
x
? 
?
%s
*synth2s
_       both_stages_valid |                               10 |                               11
2default:defaulth p
x
? 
?
%s
*synth2s
_            stage2_valid |                               11 |                               01
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys20
gen_fwft.curr_fwft_state_reg2default:default2

sequential2default:default21
xpm_fifo_base__parameterized02default:defaultZ8-3354h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_                 invalid |                               00 |                               00
2default:defaulth p
x
? 
?
%s
*synth2s
_            stage1_valid |                               01 |                               10
2default:defaulth p
x
? 
?
%s
*synth2s
_       both_stages_valid |                               10 |                               11
2default:defaulth p
x
? 
?
%s
*synth2s
_            stage2_valid |                               11 |                               01
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys20
gen_fwft.curr_fwft_state_reg2default:default2

sequential2default:default21
xpm_fifo_base__parameterized12default:defaultZ8-3354h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:19 ; elapsed = 00:00:20 . Memory (MB): peak = 3551.098 ; gain = 486.750 ; free physical = 2353 ; free virtual = 54937
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   32 Bit       Adders := 16    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   26 Bit       Adders := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input   10 Bit       Adders := 4     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   10 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    9 Bit       Adders := 5     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    9 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    8 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    6 Bit       Adders := 6     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    6 Bit       Adders := 5     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    5 Bit       Adders := 8     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    5 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    4 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    2 Bit       Adders := 3     
2default:defaulth p
x
? 
8
%s
*synth2 
+---XORs : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      1 Bit         XORs := 11    
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	              644 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	              513 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	              512 Bit    Registers := 4     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               64 Bit    Registers := 5     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               32 Bit    Registers := 5     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               26 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               10 Bit    Registers := 5     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                9 Bit    Registers := 6     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                8 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                6 Bit    Registers := 14    
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                5 Bit    Registers := 9     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                4 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                3 Bit    Registers := 3     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                2 Bit    Registers := 7     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                1 Bit    Registers := 96    
2default:defaulth p
x
? 
8
%s
*synth2 
+---RAMs : 
2default:defaulth p
x
? 
k
%s
*synth2S
?	             256K Bit	(512 X 513 bit)          RAMs := 1     
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   64 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   8 Input   32 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   32 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    8 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    6 Bit        Muxes := 4     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    4 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    4 Bit        Muxes := 4     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    3 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    3 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   8 Input    3 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    2 Bit        Muxes := 99    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    2 Bit        Muxes := 21    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    1 Bit        Muxes := 69    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    1 Bit        Muxes := 6     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    1 Bit        Muxes := 2     
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Finished RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2o
[Part Resources:
DSPs: 9024 (col length:94)
BRAMs: 4032 (col length: RAMB18 288 RAMB36 144)
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
W
%s
*synth2?
+Start Cross Boundary and Area Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2?
?RAM ("\inst_example/inst_example_vadd_m00_axi/inst_axi_read_master/gen_fifo.inst_rd_xpm_fifo_sync /xpm_fifo_base_inst/\gen_sdpram.xpm_memory_base_inst/gen_wr_a.gen_word_narrow.mem_reg ") is too shallow (depth = 512) to use URAM. Choosing BRAM instead of URAM 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:24 ; elapsed = 00:00:25 . Memory (MB): peak = 3551.098 ; gain = 486.750 ; free physical = 2335 ; free virtual = 54926
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth px? 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px? 
d
%s*synth2L
8
Block RAM: Preliminary Mapping Report (see note below)
2default:defaulth px? 
?
%s*synth2?
?+--------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+-----------------+
2default:defaulth px? 
?
%s*synth2?
?|Module Name                                                                                                                                       | RTL Object                       | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | Cascade Heights | 
2default:defaulth px? 
?
%s*synth2?
?+--------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+-----------------+
2default:defaulth px? 
?
%s*synth2?
?|\inst_example/inst_example_vadd_m00_axi/inst_axi_read_master/gen_fifo.inst_rd_xpm_fifo_sync /xpm_fifo_base_inst/\gen_sdpram.xpm_memory_base_inst  | gen_wr_a.gen_word_narrow.mem_reg | 512 x 513(READ_FIRST)  | W |   | 512 x 513(WRITE_FIRST) |   | R | Port A and B     | 1      | 7      |                 | 
2default:defaulth px? 
?
%s*synth2?
?+--------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+-----------------+

2default:defaulth px? 
?
%s*synth2?
?Note: The table above is a preliminary report that shows the Block RAMs at the current stage of the synthesis flow. Some Block RAMs may be reimplemented as non Block RAM primitives later in the synthesis flow. Multiple instantiated Block RAMs are reported only once. 
2default:defaulth px? 
j
%s*synth2R
>
Distributed RAM: Preliminary Mapping Report (see note below)
2default:defaulth px? 
?
%s*synth2?
?+------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+----------------+----------------------+----------------+
2default:defaulth px? 
?
%s*synth2?
?|Module Name                                                                                                                                     | RTL Object                       | Inference      | Size (Depth x Width) | Primitives     | 
2default:defaulth px? 
?
%s*synth2?
?+------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+----------------+----------------------+----------------+
2default:defaulth px? 
?
%s*synth2?
?|\inst_example/inst_example_vadd_m00_axi/inst_adder/inst_xpm_fifo_axis /xpm_fifo_base_inst/\gen_sdpram.xpm_memory_base_inst                      | gen_wr_a.gen_word_narrow.mem_reg | User Attribute | 32 x 644             | RAM32M16 x 46  | 
2default:defaulth px? 
?
%s*synth2?
?|\inst_example/inst_example_vadd_m00_axi/inst_axi_write_master/gen_fifo.inst_xpm_fifo_sync /xpm_fifo_base_inst/\gen_sdpram.xpm_memory_base_inst  | gen_wr_a.gen_word_narrow.mem_reg | User Attribute | 32 x 512             | RAM32M16 x 37  | 
2default:defaulth px? 
?
%s*synth2?
?+------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+----------------+----------------------+----------------+

2default:defaulth px? 
?
%s*synth2?
?Note: The table above is a preliminary report that shows the Distributed RAMs at the current stage of the synthesis flow. Some Distributed RAMs may be reimplemented as non Distributed RAM primitives later in the synthesis flow. Multiple instantiated RAMs are reported only once.
2default:defaulth px? 
?
%s*synth2?
?---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth px? 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
R
%s
*synth2:
&Start Applying XDC Timing Constraints
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:36 ; elapsed = 00:00:39 . Memory (MB): peak = 3807.723 ; gain = 743.375 ; free physical = 1821 ; free virtual = 54413
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
F
%s
*synth2.
Start Timing Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Timing Optimization : Time (s): cpu = 00:00:39 ; elapsed = 00:00:42 . Memory (MB): peak = 3856.762 ; gain = 792.414 ; free physical = 1795 ; free virtual = 54387
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2?
?---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!
Block RAM: Final Mapping Report
2default:defaulth p
x
? 
?
%s
*synth2?
?+--------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+-----------------+
2default:defaulth p
x
? 
?
%s
*synth2?
?|Module Name                                                                                                                                       | RTL Object                       | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | Cascade Heights | 
2default:defaulth p
x
? 
?
%s
*synth2?
?+--------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+-----------------+
2default:defaulth p
x
? 
?
%s
*synth2?
?|\inst_example/inst_example_vadd_m00_axi/inst_axi_read_master/gen_fifo.inst_rd_xpm_fifo_sync /xpm_fifo_base_inst/\gen_sdpram.xpm_memory_base_inst  | gen_wr_a.gen_word_narrow.mem_reg | 512 x 513(READ_FIRST)  | W |   | 512 x 513(WRITE_FIRST) |   | R | Port A and B     | 1      | 7      |                 | 
2default:defaulth p
x
? 
?
%s
*synth2?
?+--------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+-----------------+

2default:defaulth p
x
? 
S
%s
*synth2;
'
Distributed RAM: Final Mapping Report
2default:defaulth p
x
? 
?
%s
*synth2?
?+------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+----------------+----------------------+----------------+
2default:defaulth p
x
? 
?
%s
*synth2?
?|Module Name                                                                                                                                     | RTL Object                       | Inference      | Size (Depth x Width) | Primitives     | 
2default:defaulth p
x
? 
?
%s
*synth2?
?+------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+----------------+----------------------+----------------+
2default:defaulth p
x
? 
?
%s
*synth2?
?|\inst_example/inst_example_vadd_m00_axi/inst_adder/inst_xpm_fifo_axis /xpm_fifo_base_inst/\gen_sdpram.xpm_memory_base_inst                      | gen_wr_a.gen_word_narrow.mem_reg | User Attribute | 32 x 644             | RAM32M16 x 46  | 
2default:defaulth p
x
? 
?
%s
*synth2?
?|\inst_example/inst_example_vadd_m00_axi/inst_axi_write_master/gen_fifo.inst_xpm_fifo_sync /xpm_fifo_base_inst/\gen_sdpram.xpm_memory_base_inst  | gen_wr_a.gen_word_narrow.mem_reg | User Attribute | 32 x 512             | RAM32M16 x 37  | 
2default:defaulth p
x
? 
?
%s
*synth2?
?+------------------------------------------------------------------------------------------------------------------------------------------------+----------------------------------+----------------+----------------------+----------------+

2default:defaulth p
x
? 
?
%s
*synth2?
?---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-
Start Technology Mapping
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Technology Mapping : Time (s): cpu = 00:00:41 ; elapsed = 00:00:44 . Memory (MB): peak = 3864.770 ; gain = 800.422 ; free physical = 1791 ; free virtual = 54383
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2'
Start IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Q
%s
*synth29
%Start Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
T
%s
*synth2<
(Finished Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished IO Insertion : Time (s): cpu = 00:00:45 ; elapsed = 00:00:48 . Memory (MB): peak = 3864.770 ; gain = 800.422 ; free physical = 1791 ; free virtual = 54383
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Start Renaming Generated Instances
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Instances : Time (s): cpu = 00:00:45 ; elapsed = 00:00:48 . Memory (MB): peak = 3864.770 ; gain = 800.422 ; free physical = 1791 ; free virtual = 54383
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start Rebuilding User Hierarchy
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:45 ; elapsed = 00:00:49 . Memory (MB): peak = 3864.770 ; gain = 800.422 ; free physical = 1772 ; free virtual = 54364
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Renaming Generated Ports
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Ports : Time (s): cpu = 00:00:45 ; elapsed = 00:00:49 . Memory (MB): peak = 3864.770 ; gain = 800.422 ; free physical = 1772 ; free virtual = 54364
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:45 ; elapsed = 00:00:49 . Memory (MB): peak = 3864.770 ; gain = 800.422 ; free physical = 1789 ; free virtual = 54381
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
J
%s
*synth22
Start Renaming Generated Nets
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Nets : Time (s): cpu = 00:00:46 ; elapsed = 00:00:49 . Memory (MB): peak = 3864.770 ; gain = 800.422 ; free physical = 1789 ; free virtual = 54380
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Writing Synthesis Report
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
A
%s
*synth2)

Report BlackBoxes: 
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
| |BlackBox name |Instances |
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
A
%s*synth2)

Report Cell Usage: 
2default:defaulth px? 
F
%s*synth2.
+------+---------+------+
2default:defaulth px? 
F
%s*synth2.
|      |Cell     |Count |
2default:defaulth px? 
F
%s*synth2.
+------+---------+------+
2default:defaulth px? 
F
%s*synth2.
|1     |CARRY8   |    88|
2default:defaulth px? 
F
%s*synth2.
|2     |LUT1     |    84|
2default:defaulth px? 
F
%s*synth2.
|3     |LUT2     |    92|
2default:defaulth px? 
F
%s*synth2.
|4     |LUT3     |   242|
2default:defaulth px? 
F
%s*synth2.
|5     |LUT4     |    67|
2default:defaulth px? 
F
%s*synth2.
|6     |LUT5     |    86|
2default:defaulth px? 
F
%s*synth2.
|7     |LUT6     |    74|
2default:defaulth px? 
F
%s*synth2.
|8     |RAM32M16 |    83|
2default:defaulth px? 
F
%s*synth2.
|9     |RAMB18E2 |     1|
2default:defaulth px? 
F
%s*synth2.
|10    |RAMB36E2 |     7|
2default:defaulth px? 
F
%s*synth2.
|11    |FDRE     |  3842|
2default:defaulth px? 
F
%s*synth2.
|12    |FDSE     |    39|
2default:defaulth px? 
F
%s*synth2.
+------+---------+------+
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Writing Synthesis Report : Time (s): cpu = 00:00:46 ; elapsed = 00:00:49 . Memory (MB): peak = 3864.770 ; gain = 800.422 ; free physical = 1790 ; free virtual = 54382
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
r
%s
*synth2Z
FSynthesis finished with 0 errors, 0 critical warnings and 0 warnings.
2default:defaulth p
x
? 
?
%s
*synth2?
?Synthesis Optimization Runtime : Time (s): cpu = 00:00:37 ; elapsed = 00:00:41 . Memory (MB): peak = 3864.770 ; gain = 579.141 ; free physical = 1827 ; free virtual = 54419
2default:defaulth p
x
? 
?
%s
*synth2?
?Synthesis Optimization Complete : Time (s): cpu = 00:00:46 ; elapsed = 00:00:49 . Memory (MB): peak = 3864.777 ; gain = 800.422 ; free physical = 1827 ; free virtual = 54419
2default:defaulth p
x
? 
B
 Translating synthesized netlist
350*projectZ1-571h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.092default:default2
00:00:00.102default:default2
3875.7382default:default2
0.0002default:default2
19162default:default2
545072default:defaultZ17-722h px? 
g
-Analyzing %s Unisim elements for replacement
17*netlist2
1712default:defaultZ29-17h px? 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
02default:defaultZ29-28h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
3972.6762default:default2
0.0002default:default2
18092default:default2
544012default:defaultZ17-722h px? 
?
!Unisim Transformation Summary:
%s111*project2
k  A total of 83 instances were transformed.
  RAM32M16 => RAM32M16 (RAMD32(x14), RAMS32(x2)): 83 instances
2default:defaultZ1-111h px? 
g
$Synth Design complete, checksum: %s
562*	vivadotcl2
2c799a3c2default:defaultZ4-1430h px? 
U
Releasing license: %s
83*common2
	Synthesis2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1052default:default2
02default:default2
02default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
synth_design2default:defaultZ4-42h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
synth_design: 2default:default2
00:01:032default:default2
00:01:022default:default2
3972.6762default:default2
1352.6882default:default2
19692default:default2
545612default:defaultZ17-722h px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2?
?/home/wlwll6873/vitis_example_my/example_vadd_kernels/vivado_rtl_kernel/rtl_kernel_wizard_0_ex/rtl_kernel_wizard_0_ex.runs/synth_1/rtl_kernel_wizard_0.dcp2default:defaultZ17-1381h px? 
?
%s4*runtcl2?
|Executing : report_utilization -file rtl_kernel_wizard_0_utilization_synth.rpt -pb rtl_kernel_wizard_0_utilization_synth.pb
2default:defaulth px? 
?
Exiting %s at %s...
206*common2
Vivado2default:default2,
Thu Jun 30 17:58:10 20222default:defaultZ17-206h px? 


End Record