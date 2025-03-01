[ActiveSupport PAR]
; Global primary clocks
GLOBAL_PRIMARY_USED = 4;
; Global primary clock #0
GLOBAL_PRIMARY_0_SIGNALNAME = mipi2parallel_inst/rx_dphy_inst/dphy_rx_inst/clk_byte_o;
GLOBAL_PRIMARY_0_DRIVERTYPE = MIPIDPHY;
GLOBAL_PRIMARY_0_LOADNUM = 1;
; Global primary clock #1
GLOBAL_PRIMARY_1_SIGNALNAME = mipi2parallel_inst/clk_byte_hs_w;
GLOBAL_PRIMARY_1_DRIVERTYPE = MIPIDPHY;
GLOBAL_PRIMARY_1_LOADNUM = 3;
; Global primary clock #2
GLOBAL_PRIMARY_2_SIGNALNAME = clk_pixel_o_c;
GLOBAL_PRIMARY_2_DRIVERTYPE = PLL;
GLOBAL_PRIMARY_2_LOADNUM = 3;
; Global primary clock #3
GLOBAL_PRIMARY_3_SIGNALNAME = ref_clk_i_c;
GLOBAL_PRIMARY_3_DRIVERTYPE = PIO;
GLOBAL_PRIMARY_3_LOADNUM = 1;
; I/O Bank 0 Usage
BANK_0_USED = 0;
BANK_0_AVAIL = 7;
BANK_0_VCCIO = NA;
BANK_0_VREF1 = NA;
BANK_0_VREF2 = NA;
; I/O Bank 1 Usage
BANK_1_USED = 14;
BANK_1_AVAIL = 14;
BANK_1_VCCIO = 1.8V;
BANK_1_VREF1 = NA;
BANK_1_VREF2 = NA;
; I/O Bank 2 Usage
BANK_2_USED = 16;
BANK_2_AVAIL = 16;
BANK_2_VCCIO = 1.8V;
BANK_2_VREF1 = NA;
BANK_2_VREF2 = NA;
