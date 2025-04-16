library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Lab 4
entity top_basys3 is
    port(
        -- inputs
        clk     : in  std_logic;                     -- 100 MHz FPGA clock
        sw      : in  std_logic_vector(15 downto 0);
        btnU    : in  std_logic;                     -- master reset
        btnL    : in  std_logic;                     -- clock reset
        btnR    : in  std_logic;                     -- FSM reset

        -- outputs
        led     : out std_logic_vector(15 downto 0);
        seg     : out std_logic_vector(6 downto 0);   -- 7-seg segments (active-low)
        an      : out std_logic_vector(3 downto 0)    -- 7-seg anodes (active-low)
    );
end top_basys3;

architecture top_basys3_arch of top_basys3 is

    -- clock & reset signals
    signal slow_clk      : std_logic;
    signal tdm_clk       : std_logic := '0';
    signal tdm_counter   : unsigned(16 downto 0) := (others => '0');
    signal reset_clk     : std_logic;
    signal reset_fsm     : std_logic;

    -- FSM floor outputs
    signal fsm_floor1    : std_logic_vector(3 downto 0);
    signal fsm_floor2    : std_logic_vector(3 downto 0);

    -- TDM outputs
    signal mux_data      : std_logic_vector(3 downto 0);
    signal mux_sel       : std_logic_vector(3 downto 0);

    -- constants
    constant C_F         : std_logic_vector(3 downto 0) := "1111";   -- "F"
    constant TDM_COUNT   : natural := 50_000;                        -- toggle rate
    constant CLK_DIV     : natural := 25_000_000;                    -- for slow_clk

    -- component declarations
    component sevenseg_decoder is
        port(
            i_Hex   : in  std_logic_vector(3 downto 0);
            o_seg_n : out std_logic_vector(6 downto 0)
        );
    end component;

    component elevator_controller_fsm is
        port(
            i_clk      : in  std_logic;
            i_reset    : in  std_logic;
            is_stopped : in  std_logic;
            go_up_down : in  std_logic;
            o_floor    : out std_logic_vector(3 downto 0)
        );
    end component;

    component TDM4 is
        generic ( k_WIDTH : natural := 4 );
        port(
            i_clk   : in  std_logic;
            i_reset : in  std_logic;
            i_D3    : in  std_logic_vector(k_WIDTH-1 downto 0);
            i_D2    : in  std_logic_vector(k_WIDTH-1 downto 0);
            i_D1    : in  std_logic_vector(k_WIDTH-1 downto 0);
            i_D0    : in  std_logic_vector(k_WIDTH-1 downto 0);
            o_data  : out std_logic_vector(k_WIDTH-1 downto 0);
            o_sel   : out std_logic_vector(3 downto 0)
        );
    end component;

    component clock_divider is
        generic ( k_DIV : natural := CLK_DIV );
        port(
            i_clk   : in  std_logic;
            i_reset : in  std_logic;
            o_clk   : out std_logic
        );
    end component;

begin
    -- reset logic
    reset_clk <= btnU or btnL;
    reset_fsm <= btnU or btnR;

    -- generate slow_clk (~2 Hz)
    U_CLK_DIV: clock_divider
        generic map(k_DIV => CLK_DIV)
        port map(
            i_clk   => clk,
            i_reset => reset_clk,
            o_clk   => slow_clk
        );

    -- generate TDM clock by counting system clk cycles
    process(clk)
    begin
        if rising_edge(clk) then
            if tdm_counter = to_unsigned(TDM_COUNT-1, tdm_counter'length) then
                tdm_counter <= (others => '0');
                tdm_clk     <= not tdm_clk;
            else
                tdm_counter <= tdm_counter + 1;
            end if;
        end if;
    end process;

    -- elevator #1 (sw(1)=up, sw(0)=stop)
    U_ELEV1: elevator_controller_fsm
        port map(
            i_clk      => slow_clk,
            i_reset    => reset_fsm,
            is_stopped => sw(0),
            go_up_down => sw(1),
            o_floor    => fsm_floor1
        );

    -- elevator #2 (sw(15)=up, sw(14)=stop)
    U_ELEV2: elevator_controller_fsm
        port map(
            i_clk      => slow_clk,
            i_reset    => reset_fsm,
            is_stopped => sw(14),
            go_up_down => sw(15),
            o_floor    => fsm_floor2
        );

    -- time-division multiplexing
    U_TDM4: TDM4
        generic map(k_WIDTH => 4)
        port map(
            i_clk   => tdm_clk,
            i_reset => btnU,
            i_D3    => C_F,
            i_D2    => fsm_floor2,
            i_D1    => C_F,
            i_D0    => fsm_floor1,
            o_data  => mux_data,
            o_sel   => mux_sel
        );

    -- seven-seg decode
    U_SEVENSEG: sevenseg_decoder
        port map(
            i_Hex   => mux_data,
            o_seg_n => seg
        );

    -- drive LEDs and anodes
    led(15)        <= slow_clk;
    led(3 downto 0)  <= fsm_floor1;
    led(7 downto 4)  <= fsm_floor2;
    led(14 downto 8) <= (others => '0');
    an             <= not mux_sel;   -- active-low anodes

end top_basys3_arch;

