# build_bd.tcl — KR260 Zynq US+ PS + reverb_kernel HLS IP + FAN route (TTC0->EMIO->A12).
# Adapted from the CRITICALITY M3 build; fan route INVERTED-polarity (A12/LVCMOS33) intact.
set root   $::env(HOME)/kria-reverb
set part   xck26-sfvc784-2LV-c
set board  xilinx.com:kr260_som:part0:1.1

create_project -force reverb_bd $root/reverb_bd -part $part
set_property board_part $board [current_project]
set_property ip_repo_paths $root/reverb_ip/sol1/impl/ip [current_fileset]
update_ip_catalog

create_bd_design "design_1"

# --- PS (board preset) + TTC0 WAVEOUT->EMIO for fan PWM ---
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:* ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset 1} $ps
set_property -dict [list \
  CONFIG.PSU__FPGA_PL0_ENABLE {1} \
  CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {150} \
  CONFIG.PSU__USE__M_AXI_GP0 {1} \
  CONFIG.PSU__USE__M_AXI_GP1 {0} \
  CONFIG.PSU__USE__M_AXI_GP2 {0} \
  CONFIG.PSU__USE__S_AXI_GP2 {1} \
  CONFIG.PSU__TTC0__WAVEOUT__ENABLE {1} \
  CONFIG.PSU__TTC0__WAVEOUT__IO {EMIO} \
] $ps

# --- the kernel ---
set k [create_bd_cell -type ip -vlnv xilinx.com:hls:reverb_kernel:* k]

# --- smartconnects (ctrl 1->1 ; data 2->1 : gmem0 + gmem1) ---
set scc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:* sc_ctrl]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $scc
set scd [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:* sc_data]
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] $scd

# --- reset ---
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:* rst]

# --- clock (pl_clk0) + reset wiring ---
set clk  [get_bd_pins ps/pl_clk0]
set rstn [get_bd_pins ps/pl_resetn0]
connect_bd_net $clk  [get_bd_pins rst/slowest_sync_clk]
connect_bd_net $rstn [get_bd_pins rst/ext_reset_in]
set presetn [get_bd_pins rst/peripheral_aresetn]
foreach p {ps/maxihpm0_fpd_aclk ps/saxihp0_fpd_aclk sc_ctrl/aclk sc_data/aclk k/ap_clk} {
  connect_bd_net $clk [get_bd_pins $p]
}
foreach p {sc_ctrl/aresetn sc_data/aresetn k/ap_rst_n} {
  connect_bd_net $presetn [get_bd_pins $p]
}

# --- AXI: control + data ---
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins sc_ctrl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M00_AXI]   [get_bd_intf_pins k/s_axi_ctrl]
connect_bd_intf_net [get_bd_intf_pins k/m_axi_gmem0]   [get_bd_intf_pins sc_data/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins k/m_axi_gmem1]   [get_bd_intf_pins sc_data/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_data/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]

# --- FAN route: TTC0 wave -> slice bit0 -> INVERT -> external port fan_en_b (A12) ---
set FAN_WAVE_BIT 0
set wavepin ""
foreach p [get_bd_pins ps/*] { if {[string match -nocase *ttc0*wave* $p]} { set wavepin $p } }
puts "FAN_WAVEPIN=$wavepin"
if {$wavepin eq ""} { error "TTC0 wave pin not found - PS TTC0 EMIO not enabled" }
set slice [create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:* fan_slice]
set_property -dict [list CONFIG.DIN_WIDTH {3} CONFIG.DIN_FROM $FAN_WAVE_BIT CONFIG.DIN_TO $FAN_WAVE_BIT CONFIG.DOUT_WIDTH {1}] $slice
connect_bd_net [get_bd_pins $wavepin] [get_bd_pins fan_slice/Din]
set fnot [create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic:* fan_not]
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {not}] $fnot
connect_bd_net [get_bd_pins fan_slice/Dout] [get_bd_pins fan_not/Op1]
create_bd_port -dir O fan_en_b
connect_bd_net [get_bd_pins fan_not/Res] [get_bd_ports fan_en_b]
puts "FAN_POLARITY=inverted FAN_WAVE_BIT=$FAN_WAVE_BIT"
set clkin ""
foreach p [get_bd_pins ps/*] { if {[string match -nocase *ttc0*clk_i* $p]} { set clkin $p } }
puts "FAN_CLKIN=$clkin"
if {$clkin ne ""} {
  set zero [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:* fan_zero]
  set_property -dict [list CONFIG.CONST_WIDTH {3} CONFIG.CONST_VAL {0}] $zero
  connect_bd_net [get_bd_pins fan_zero/dout] [get_bd_pins $clkin]
}

assign_bd_address
validate_bd_design
save_bd_design
puts "BD_VALIDATE_OK"
