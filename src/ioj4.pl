#!/usr/bin/perl
use strict;
use warnings;
use Text::CSV;
use POSIX qw(ceil floor);
use List::Util qw(min max sum shuffle);
use Math::Random qw(random_normal);
use Time::HiRes qw(gettimeofday);
use Data::Dumper;
use Statistics::Basic qw(mean stddev);
use Statistics::Descriptive;
use Scalar::Util qw(looks_like_number);

##############################################################################
# Enhanced LoRaWAN Gateway Optimization Simulator with Realistic Constraints #
##############################################################################

###############################  Parameters  #################################
# Optimization parameters
my $POP_SIZE = 30;
my $COATI_ITERS = 50;
my $MUTATION_RATE = 0.15;
my $OVERLOAD_THRESH = 100;

my $GA_POP_SIZE = 30;
my $GA_ITERS = 50;
my $GA_MUT = 0.15;

my $PSO_POP_SIZE = 30;
my $PSO_ITERS = 50;

# Enhanced optimization parameters
my $LAMBDA_FAIR = 0.15;
my $LAMBDA_P = 0.2;
my $STAGNATION_THRESHOLD = 10;
my $RESTART_RATE = 0.15;

# Physical layer parameters
my @SF_SENSITIVITY = (-124, -127, -130, -133, -135, -137);
my @SNR_THRESHOLDS = (-7.5, -10, -12.5, -15, -17.5, -20);
my @CAPTURE_THRESHOLDS = (
    [1, -8, -9, -9, -9, -9],
    [-11, 1, -11, -12, -13, -13],
    [-15, -13, 1, -13, -14, -15],
    [-19, -18, -17, 1, -17, -18],
    [-22, -22, -21, -20, 1, -20],
    [-25, -25, -25, -24, -23, 1]
);
my $NOISE_FLOOR = -120;
my $LPLD0 = 110;
my $GAMMA_PL = 2.08;
my $DREF = 40;
my $ADR_MARGIN = 5;
my @TX_POWERS = (2, 5, 8, 11, 14, 17, 20);
my $MAX_SF = 12;
my $MIN_SF = 7;

# LoRaWAN parameters
my $DEFAULT_POWER = 14;
my @CHANNELS = (868.1, 868.3, 868.5);
my %DR_PAYLOAD_SIZE = (
    DR0=>51, DR1=>51, DR2=>51, DR3=>115, DR4=>242, DR5=>222
);
my %DR_TO_SF = (
    DR0 => 12, DR1 => 11, DR2 => 10, DR3 => 9, DR4 => 8, DR5 => 7
);
my %SF_TO_DR = reverse %DR_TO_SF;

# RX Window parameters
my $RX1_DELAY = 1;
my $RX2_DELAY = 2;
my $RX1_DURATION = 1;
my $RX2_DURATION = 1;
my %RX1_DR_OFFSETS = (
    DR0 => 'DR0', DR1 => 'DR0', DR2 => 'DR1', DR3 => 'DR2', 
    DR4 => 'DR3', DR5 => 'DR4'
);

# Duty cycle parameters
my %DUTY_CYCLE_LIMITS = (
    868.1 => 0.01, 868.3 => 0.01, 868.5 => 0.01
);
my $DUTY_CYCLE_WINDOW = 3600;

# Energy model parameters
my $CURRENT_TX = 120;
my $CURRENT_RX = 15;
my $CURRENT_IDLE = 5;
my $CURRENT_SLEEP = 0.1;
my $VOLTAGE = 3.3;

# ADR parameters
my $ADR_ACK_LIMIT = 64;
my $ADR_ACK_DELAY = 32;

# Energy and latency model
my $BASE_E_PER_PKT = 0.05;
my $RETRY_PENALTY = 0.02;
my $BASE_LATENCY = 10;
my $LOAD_LAT_FACTOR = 5;
my $DIST_LAT_FACTOR = 2;
my $MAX_RETRIES = 3;

# Scenario parameters
my %SCENARIOS = (
    'Industrial' => {
        density=>1.0, 
        placement=>'close', 
        interference=>0.5,
        default_dr=>'DR3',
        confirmed_ratio=>0.8
    },
    'Suburban' => {
        density=>1.0, 
        placement=>'moderate', 
        interference=>0.2,
        default_dr=>'DR2',
        confirmed_ratio=>0.4
    },
    'Rural-Agricultural'=> {
        density=>1.0, 
        placement=>'far', 
        interference=>0.05,
        default_dr=>'DR0',
        confirmed_ratio=>0.1
    },
);

# Weight configurations
my %WEIGHT_CONFIGS = (
    'Favor_RSSI' => {ALPHA=>0.6, BETA=>0.1, GAMMA=>0.2, DELTA=>0.1},
    'Favor_Load_Balancing' => {ALPHA=>0.1, BETA=>0.6, GAMMA=>0.2, DELTA=>0.1},
    'Favor_SNR' => {ALPHA=>0.2, BETA=>0.2, GAMMA=>0.5, DELTA=>0.1},
    'Favor_Interference_Avoidance' => {ALPHA=>0.3, BETA=>0.3, GAMMA=>0.1, DELTA=>0.3},
    'Balanced' => {ALPHA=>0.25, BETA=>0.25, GAMMA=>0.25, DELTA=>0.25}
);

# CLI options with defaults
my %OPTIONS = (
    seed => int(rand(10000)),
    iters => 10,
    alg => 'all',
    scenario => 'Suburban',
    density => 1.0,
    gw_count => 5,
    interf => undef,
    traffic => 'periodic',
    confirmed => undef,
    weight_config => 'Balanced',
    help => 0
);

# Weight configuration
my %weights;
my $selected_weight_id;

###############################  Data Structures  ###########################
my %rssi_history;
my %adr_updates;
my %current_sf;
my %current_power;
my %airtimes;
my %duty_cycles;
my %adr_ack_counters;

# Gateway state tracking
my %gw_busy_until;
my %gw_duty_cycles;
my %gw_last_tx;

# Signal tracking for collision detection
my %active_signals;

# Device energy state tracking
my %device_energy;
my %device_last_state;

# ACK tracking
my %ack_events;
my %ul_events;
my %retx_count;

# Statistical results
my %results;

# Global variables
my $NUM_DEV = 100;
my $NUM_GW = 5;
my $MAX_PAYLOAD = 51;

###############################  CLI Processing  ############################
sub parse_cli {
    use Getopt::Long;
    
    GetOptions(
        'seed=i'      => \$OPTIONS{seed},
        'iters=i'     => \$OPTIONS{iters},
        'alg=s'       => \$OPTIONS{alg},
        'scenario=s'  => \$OPTIONS{scenario},
        'density=f'   => \$OPTIONS{density},
        'gw_count=i'  => \$OPTIONS{gw_count},
        'interf=f'    => \$OPTIONS{interf},
        'traffic=s'   => \$OPTIONS{traffic},
        'confirmed=f' => \$OPTIONS{confirmed},
        'weight_config=s' => \$OPTIONS{weight_config},
        'help'        => \$OPTIONS{help}
    ) or die "Error in command line arguments\n";
    
    if ($OPTIONS{help}) {
        print "Usage: $0 [options]\n";
        print "Options:\n";
        print "  --seed N          Random seed (default: random)\n";
        print "  --iters N         Number of iterations (default: 10)\n";
        print "  --alg ALG         Algorithm to test (default: all)\n";
        print "  --scenario NAME   Scenario (Industrial, Suburban, Rural-Agricultural)\n";
        print "  --density F       Device density multiplier (default: 1.0)\n";
        print "  --gw_count N      Number of gateways (default: 5)\n";
        print "  --interf F        Interference level (0.0-1.0, default: scenario)\n";
        print "  --traffic TYPE    Traffic pattern (periodic, bursty, default: periodic)\n";
        print "  --confirmed F     Ratio of confirmed traffic (0.0-1.0, default: scenario)\n";
        print "  --weight_config C Weight configuration (Favor_RSSI, Favor_Load_Balancing, Favor_SNR, Favor_Interference_Avoidance, Balanced)\n";
        print "  --help            Show this help\n";
        exit 0;
    }
    
    # Validate scenario
    unless (exists $SCENARIOS{$OPTIONS{scenario}}) {
        die "Invalid scenario: $OPTIONS{scenario}. Choose from: " . 
            join(', ', keys %SCENARIOS) . "\n";
    }
    
    # Validate weight configuration
    unless (exists $WEIGHT_CONFIGS{$OPTIONS{weight_config}}) {
        die "Invalid weight configuration: $OPTIONS{weight_config}. Choose from: " . 
            join(', ', keys %WEIGHT_CONFIGS) . "\n";
    }
    
    # Set weight configuration
    %weights = %{$WEIGHT_CONFIGS{$OPTIONS{weight_config}}};
    $selected_weight_id = $OPTIONS{weight_config};
    
    # Set defaults from scenario if not provided
    $OPTIONS{interf} = $SCENARIOS{$OPTIONS{scenario}}{interference} 
        unless defined $OPTIONS{interf};
    $OPTIONS{confirmed} = $SCENARIOS{$OPTIONS{scenario}}{confirmed_ratio}
        unless defined $OPTIONS{confirmed};
        
    # Set random seed
    srand($OPTIONS{seed});
    
    # Set device and gateway counts based on scenario density
    $NUM_DEV = int(100 * $OPTIONS{density});
    $NUM_GW = $OPTIONS{gw_count};
    
    print "Configuration:\n";
    foreach my $key (sort keys %OPTIONS) {
        next if $key eq 'help';
        printf "  %-15s: %s\n", $key, $OPTIONS{$key};
    }
    printf "  %-15s: %s\n", "weight_config_details", 
        "ALPHA=$weights{ALPHA}, BETA=$weights{BETA}, GAMMA=$weights{GAMMA}, DELTA=$weights{DELTA}";
    print "\n";
}

###############################  RX Window Scheduling #######################
sub compute_rx_windows {
    my ($t_ul_end, $ul_dr, $rx1_offset) = @_;
    
    # Calculate RX1 and RX2 start times
    my $rx1_start = $t_ul_end + $RX1_DELAY;
    my $rx2_start = $t_ul_end + $RX2_DELAY;
    
    # Calculate RX data rates
    my $rx1_dr = rx1_dr_from_ul($ul_dr, $rx1_offset);
    my $rx2_dr = 'DR0';  # Fixed for RX2 in EU868
    
    return (
        { start => $rx1_start, end => $rx1_start + $RX1_DURATION, dr => $rx1_dr },
        { start => $rx2_start, end => $rx2_start + $RX2_DURATION, dr => $rx2_dr }
    );
}

sub rx1_dr_from_ul {
    my ($ul_dr, $offset) = @_;
    
    # Convert DR string to numeric value
    my $ul_dr_num = substr($ul_dr, 2);
    my $rx1_dr_num = max(0, $ul_dr_num - $offset);
    
    return "DR$rx1_dr_num";
}

###############################  Duty Cycle Enforcement #####################
sub init_gw_duty {
    my ($gw_id) = @_;
    
    foreach my $channel (@CHANNELS) {
        $gw_duty_cycles{$gw_id}{$channel} = {
            used_time => 0,
            last_reset => 0,
            window_start => 0
        };
    }
}

sub reset_duty_cycle_windows {
    my ($current_time) = @_;
    
    for my $gw_id (0..$NUM_GW-1) {
        for my $channel (@CHANNELS) {
            my $duty_cycle = $gw_duty_cycles{$gw_id}{$channel};
            
            # Check if we need to reset the duty cycle window
            if ($current_time - $duty_cycle->{window_start} >= $DUTY_CYCLE_WINDOW) {
                $duty_cycle->{used_time} = 0;
                $duty_cycle->{window_start} = $current_time;
                $duty_cycle->{last_reset} = $current_time;
            }
        }
    }
}

sub gw_can_tx_at {
    my ($gw_id, $channel, $t_start, $airtime) = @_;
    
    # Check half-duplex (gateway busy)
    if (exists $gw_busy_until{$gw_id} && $t_start < $gw_busy_until{$gw_id}) {
        return 0;
    }
    
    # Check duty cycle
    my $duty_cycle = $gw_duty_cycles{$gw_id}{$channel};
    
    # Reset duty cycle window if needed
    if ($t_start - $duty_cycle->{window_start} >= $DUTY_CYCLE_WINDOW) {
        $duty_cycle->{used_time} = 0;
        $duty_cycle->{window_start} = $t_start;
        $duty_cycle->{last_reset} = $t_start;
    }
    
    my $window_start = max($duty_cycle->{last_reset}, $t_start - $DUTY_CYCLE_WINDOW);
    my $used_in_window = $duty_cycle->{used_time};
    
    # Reset if window has advanced
    if ($t_start - $duty_cycle->{last_reset} > $DUTY_CYCLE_WINDOW) {
        $used_in_window = 0;
        $duty_cycle->{last_reset} = $t_start;
    }
    
    my $allowed_time = $DUTY_CYCLE_LIMITS{$channel} * $DUTY_CYCLE_WINDOW;
    if ($used_in_window + $airtime > $allowed_time) {
        return 0;
    }
    
    return 1;
}

sub apply_duty_backoff {
    my ($gw_id, $channel, $t_start, $airtime) = @_;
    
    my $duty_cycle = $gw_duty_cycles{$gw_id}{$channel};
    $duty_cycle->{used_time} += $airtime;
    
    # Update gateway busy until time
    $gw_busy_until{$gw_id} = $t_start + $airtime;
}

sub pick_legal_subband_for_time {
    my ($gw_id, $t_win_start, $t_win_end) = @_;
    
    my $best_channel;
    my $best_start;
    my $min_delay = 1e9;
    
    foreach my $channel (@CHANNELS) {
        # Find earliest available time in this channel
        my $available_time = $t_win_start;
        
        # Check duty cycle constraints
        my $duty_cycle = $gw_duty_cycles{$gw_id}{$channel};
        
        # Reset duty cycle window if needed
        if ($available_time - $duty_cycle->{window_start} >= $DUTY_CYCLE_WINDOW) {
            $duty_cycle->{used_time} = 0;
            $duty_cycle->{window_start} = $available_time;
            $duty_cycle->{last_reset} = $available_time;
        }
        
        my $window_start = max($duty_cycle->{last_reset}, $available_time - $DUTY_CYCLE_WINDOW);
        my $used_in_window = $duty_cycle->{used_time};
        
        # Reset if window has advanced
        if ($available_time - $duty_cycle->{last_reset} > $DUTY_CYCLE_WINDOW) {
            $used_in_window = 0;
            $duty_cycle->{last_reset} = $available_time;
        }
        
        my $allowed_time = $DUTY_CYCLE_LIMITS{$channel} * $DUTY_CYCLE_WINDOW;
        my $remaining_time = $allowed_time - $used_in_window;
        
        # If we don't have enough time, find when we will
        if ($remaining_time < ($t_win_end - $t_win_start)) {
            # Calculate when enough time will be available
            my $time_needed = ($t_win_end - $t_win_start) - $remaining_time;
            $available_time += $time_needed / $DUTY_CYCLE_LIMITS{$channel};
        }
        
        # Check half-duplex
        if (exists $gw_busy_until{$gw_id} && $available_time < $gw_busy_until{$gw_id}) {
            $available_time = $gw_busy_until{$gw_id};
        }
        
        # Check if this fits in the window
        if ($available_time <= $t_win_end) {
            my $delay = $available_time - $t_win_start;
            if ($delay < $min_delay) {
                $min_delay = $delay;
                $best_channel = $channel;
                $best_start = $available_time;
            }
        }
    }
    
    return ($best_channel, $best_start);
}

###############################  Gateway Selection for ACK ##################
sub predicted_dl_snr_margin {
    my ($dev_id, $gw_id, $dr) = @_;
    
    # Use historical RSSI to predict DL SNR
    my $hist = $rssi_history{$dev_id}{$gw_id} || [];
    my $avg_rssi = @$hist ? sum(@$hist)/@$hist : -120;
    
    my $sf = $DR_TO_SF{$dr};
    my $sensitivity = $SF_SENSITIVITY[$sf-7];
    my $snr_margin = $avg_rssi - $sensitivity;
    
    return $snr_margin;
}

sub score_gateway_for_window {
    my ($dev_id, $gw_id, $window, $current_load) = @_;
    
    my $score = 0;
    
    # Check if gateway can transmit in this window
    my ($channel, $start_time) = pick_legal_subband_for_time(
        $gw_id, $window->{start}, $window->{end}
    );
    
    return -1e9 unless defined $channel;  # Cannot use this gateway
    
    # Calculate SNR margin
    my $snr_margin = predicted_dl_snr_margin($dev_id, $gw_id, $window->{dr});
    
    # Load factor (lower is better)
    my $load_factor = $current_load->{$gw_id} / ($OVERLOAD_THRESH || 1);
    $load_factor = min(1, $load_factor);
    
    # Delay factor (earlier is better)
    my $delay = $start_time - $window->{start};
    my $max_delay = $window->{end} - $window->{start};
    my $delay_factor = 1 - ($delay / $max_delay);
    
    # Combined score
    $score = $snr_margin * 0.5 +        # SNR margin (higher better)
             $delay_factor * 0.3 +      # Timeliness (higher better)
             (1 - $load_factor) * 0.2;  # Load balance (higher better)
    
    return $score;
}

sub select_gw_for_ack {
    my ($dev_id, $t_ul_end, $ul_dr, $rx1_offset) = @_;
    
    # Compute RX windows
    my ($rx1_win, $rx2_win) = compute_rx_windows($t_ul_end, $ul_dr, $rx1_offset);
    
    # Get current gateway loads
    my %gw_loads;
    for my $gw_id (0..$NUM_GW-1) {
        $gw_loads{$gw_id} = 0;  # Would be updated from global state
    }
    
    # Score gateways for RX1
    my %rx1_scores;
    my $best_rx1_score = -1e9;
    my $best_rx1_gw;
    
    for my $gw_id (0..$NUM_GW-1) {
        my $score = score_gateway_for_window($dev_id, $gw_id, $rx1_win, \%gw_loads);
        $rx1_scores{$gw_id} = $score;
        
        if ($score > $best_rx1_score) {
            $best_rx1_score = $score;
            $best_rx1_gw = $gw_id;
        }
    }
    
    # If we found a feasible gateway for RX1, use it
    if ($best_rx1_score > -1e9) {
        return {
            gw => $best_rx1_gw,
            window => $rx1_win,
            type => 'RX1'
        };
    }
    
    # Fall back to RX2
    my %rx2_scores;
    my $best_rx2_score = -1e9;
    my $best_rx2_gw;
    
    for my $gw_id (0..$NUM_GW-1) {
        my $score = score_gateway_for_window($dev_id, $gw_id, $rx2_win, \%gw_loads);
        $rx2_scores{$gw_id} = $score;
        
        if ($score > $best_rx2_score) {
            $best_rx2_score = $score;
            $best_rx2_gw = $gw_id;
        }
    }
    
    if ($best_rx2_score > -1e9) {
        return {
            gw => $best_rx2_gw,
            window => $rx2_win,
            type => 'RX2'
        };
    }
    
    # No feasible gateway found
    return undef;
}

sub schedule_ack_tx {
    my ($gw_id, $channel, $window, $airtime) = @_;
    
    my $tx_time = $window->{start};
    
    # Apply duty cycle constraints
    unless (gw_can_tx_at($gw_id, $channel, $tx_time, $airtime)) {
        # Find next available time
        ($channel, $tx_time) = pick_legal_subband_for_time(
            $gw_id, $window->{start}, $window->{end}
        );
        
        return undef unless defined $channel;
    }
    
    # Apply duty cycle backoff
    apply_duty_backoff($gw_id, $channel, $tx_time, $airtime);
    
    return {
        gw => $gw_id,
        channel => $channel,
        start => $tx_time,
        end => $tx_time + $airtime
    };
}

###############################  Collision and Capture ######################
sub register_signal {
    my ($gw_id, $signal_id, $start_time, $end_time, $power, $sf, $dev_id) = @_;
    
    $active_signals{$gw_id}{$signal_id} = {
        start => $start_time,
        end => $end_time,
        power => $power,
        sf => $sf,
        dev => $dev_id
    };
}

sub resolve_overlaps {
    my ($gw_id, $end_time) = @_;
    
    my @signals = values %{$active_signals{$gw_id} || {}};
    
    # Remove finished signals
    foreach my $sig_id (keys %{$active_signals{$gw_id} || {}}) {
        if ($active_signals{$gw_id}{$sig_id}{end} <= $end_time) {
            delete $active_signals{$gw_id}{$sig_id};
        }
    }
    
    # Find overlapping signals
    my @overlapping;
    foreach my $sig (@signals) {
        if ($sig->{start} <= $end_time && $sig->{end} >= $end_time) {
            push @overlapping, $sig;
        }
    }
    
    return @overlapping if @overlapping <= 1;
    
    # Sort by power (descending)
    @overlapping = sort { $b->{power} <=> $a->{power} } @overlapping;
    
    my $strongest = shift @overlapping;
    my $success = 1;
    
    # Check capture effect for each weaker signal
    foreach my $weaker (@overlapping) {
        my $power_diff = $strongest->{power} - $weaker->{power};
        my $threshold = $CAPTURE_THRESHOLDS[$strongest->{sf}-7][$weaker->{sf}-7];
        
        if ($power_diff < $threshold) {
            $success = 0;
            last;
        }
    }
    
    return $success;
}
###############################  Energy Accounting ##########################
sub enter_state {
    my ($dev_id, $new_state, $current_time) = @_;
    
    my $last_state = $device_last_state{$dev_id};
    
    if ($last_state) {
        my $duration = $current_time - $last_state->{start_time};
        $device_energy{$dev_id}{$last_state->{state}} += $duration;
    }
    
    $device_last_state{$dev_id} = {
        state => $new_state,
        start_time => $current_time
    };
}

sub calculate_energy_consumption {
    my ($dev_id, $end_time) = @_;
    
    # Ensure final state is accounted for
    if (my $last_state = $device_last_state{$dev_id}) {
        my $duration = $end_time - $last_state->{start_time};
        $device_energy{$dev_id}{$last_state->{state}} += $duration;
    }
    
    # Calculate total energy
    my $total_energy = 0;
    foreach my $state (keys %{$device_energy{$dev_id} || {}}) {
        my $current = 0;
        if ($state eq 'TX') {
            $current = $CURRENT_TX;
        } elsif ($state eq 'RX') {
            $current = $CURRENT_RX;
        } elsif ($state eq 'IDLE') {
            $current = $CURRENT_IDLE;
        } elsif ($state eq 'SLEEP') {
            $current = $CURRENT_SLEEP;
        }
        
        $total_energy += $device_energy{$dev_id}{$state} * $current * $VOLTAGE / 3600;  # mJ
    }
    
    return $total_energy;
}

###############################  ADR-ACK Counters ###########################
sub adr_ack_update {
    my ($dev_id, $got_downlink) = @_;
    
    $adr_ack_counters{$dev_id} ||= { counter => 0, pending => 0 };
    my $counter = $adr_ack_counters{$dev_id};
    
    if ($got_downlink) {
        $counter->{counter} = 0;
        $counter->{pending} = 0;
    } else {
        $counter->{counter}++;
        
        if ($counter->{counter} >= $ADR_ACK_LIMIT) {
            $counter->{pending} = 1;
        }
        
        if ($counter->{counter} >= $ADR_ACK_LIMIT + $ADR_ACK_DELAY) {
            # Reset ADR to more conservative settings
            my $current_sf = $current_sf{$dev_id} || $MIN_SF;
            my $current_power = $current_power{$dev_id} || $DEFAULT_POWER;
            
            $current_sf{$dev_id} = min($MAX_SF, $current_sf + 2);
            $current_power{$dev_id} = min($TX_POWERS[-1], $current_power + 6);
            
            $counter->{counter} = 0;
            $counter->{pending} = 0;
        }
    }
    
    return $counter->{pending};
}

###############################  ADR Algorithm ##############################
sub adapt_data_rate {
    my ($dev_id, $gw_id, $current_sf, $current_power, $rssi_history) = @_;
    
    # Calculate average RSSI and margin
    my $avg_rssi = mean(@$rssi_history);
    my $required_rssi = $SF_SENSITIVITY[$current_sf-7] + $ADR_MARGIN;
    my $margin = $avg_rssi - $required_rssi;
    
    # Adjust SF and power based on margin
    my $new_sf = $current_sf;
    my $new_power = $current_power;
    
    if ($margin > 10) {
        # Strong signal, can increase data rate (lower SF)
        $new_sf = max($MIN_SF, $current_sf - 1);
    } elsif ($margin < 5) {
        # Weak signal, need to decrease data rate (increase SF)
        $new_sf = min($MAX_SF, $current_sf + 1);
        
        # If already at max SF, increase power
        if ($new_sf == $current_sf && $current_power < $TX_POWERS[-1]) {
            $new_power = min($TX_POWERS[-1], $current_power + 3);
        }
    }
    
    return ($new_sf, $new_power);
}

###############################  Helper Functions ###########################
sub calculate_rssi {
    my ($dist, $freq, $tx_pow) = @_;
    my $path_loss = $LPLD0 + 10 * $GAMMA_PL * log($dist/$DREF)/log(10);
    my $rssi = $tx_pow - $path_loss + random_normal(1, 0, 3.57);
    return $rssi - ($OPTIONS{interf}*20);
}

sub calculate_snr {
    my ($rssi) = @_;
    return $rssi - $NOISE_FLOOR;
}

sub airtime {
    my ($sf, $bw, $payload) = @_;
    my $Tsym = (2**$sf)/$bw;
    my $Tpream = (8 + 4.25)*$Tsym;
    
    my $payloadSymb = 8 + max(
        ceil(
            (8*$payload - 4*$sf + 28 + 16) / 
            (4*($sf - 2*($sf >= 11)))
        )
    ) * (5/4);
    
    return $Tpream + $payloadSymb * $Tsym;
}

sub transmission_success {
    my ($rssi, $sf, $interference) = @_;
    my $snr = calculate_snr($rssi);
    my $required_snr = $SNR_THRESHOLDS[$sf-7] + $ADR_MARGIN;
    
    my $interf_dB = $interference * 5;
    $snr -= $interf_dB;
    
    my $success_prob = 1 / (1 + exp(-0.5*($snr - $required_snr)));
    $success_prob = max(0.1, $success_prob);
    
    return ($success_prob >= rand());
}

sub transmission_success_probability {
    my ($rssi, $sf, $interf) = @_;
    my $snr = calculate_snr($rssi);
    my $required_snr = $SNR_THRESHOLDS[$sf-7] + $ADR_MARGIN;
    
    my $interf_dB = $interf * 5;
    $snr -= $interf_dB;
    
    my $success_prob = 1 / (1 + exp(-0.5*($snr - $required_snr)));
    return max(0.1, min(1.0, $success_prob));
}

# FIXED: Renamed to avoid conflict with Statistics::Basic::mean
sub _mean {
    my @values = @_;
    return sum(@values)/@values;
}

sub segment_traffic {
    my ($bytes, $max) = @_; 
    return int($bytes/$max) + (($bytes%$max)?1:0);
}

sub _minmax_mat {
    my ($mat) = @_;
    my ($mn,$mx) = (1e9,-1e9);
    for my $row (@$mat){
        for my $v (@$row){
            next unless defined $v;
            $mn = $v if $v < $mn;
            $mx = $v if $v > $mx;
        }
    }
    $mn = 0   if $mn  >  0  && $mx == 0;
    $mx = $mn+1e-9 if $mx <= $mn;
    return ($mn,$mx);
}

sub _chrom_to_vec {
    my ($chrom) = @_;
    my @x = (0) x ($NUM_DEV*$NUM_GW);
    for my $d (0..$NUM_DEV-1) {
        my $g = int($chrom->[$d] + 0.5);
        $g = 0 if $g < 0; $g = $NUM_GW-1 if $g > $NUM_GW-1;
        $x[$d*$NUM_GW + $g] = 1;
    }
    return \@x;
}

sub ga_fitness  {
    my ($c, $rssi, $snr, $interf, $chan_used) = @_;
    return fitness(_chrom_to_vec($c), $rssi, $snr, undef, $interf, $chan_used);
}

sub pso_fitness {
    my ($p, $rssi, $snr, $interf, $chan_used) = @_;
    return fitness(_chrom_to_vec($p), $rssi, $snr, undef, $interf, $chan_used);
}

sub fitness {
    my ($sol,$rssi,$snr,$traffic,$interf,$chan_used) = @_;
    my $fit = 0;

    # ---- decode implied assignment via argmax ----
    my @gw_choice; my @gw_load = (0) x $NUM_GW;
    for my $d (0 .. $NUM_DEV-1){
        my $base = $d * $NUM_GW;
        my ($sel_gw,$bestv) = (0, -9e9);
        for my $g (0 .. $NUM_GW-1){
            my $v = $sol->[$base + $g];
            if ($v > $bestv){ $bestv = $v; $sel_gw = $g; }
        }
        $gw_choice[$d] = $sel_gw;
        $gw_load[$sel_gw]++;
    }

    # Calculate fairness term (load variance)
    my $mean_load = sum(@gw_load) / ($NUM_GW || 1);
    my $var_load = 0; 
    $var_load += ($_ - $mean_load)**2 for @gw_load;
    $var_load /= ($NUM_GW || 1);
    my $fairness_penalty = $LAMBDA_FAIR * ($var_load / ($OVERLOAD_THRESH**2));

    # ---- empirical min-max ----
    my ($RSSI_MIN,$RSSI_MAX) = _minmax_mat($rssi);
    my ($SNR_MIN,$SNR_MAX)   = _minmax_mat($snr);

    for my $d (0 .. $NUM_DEV-1){
        my $g = $gw_choice[$d];

        my $norm_rssi = ($rssi->[$d][$g] - $RSSI_MIN) / (($RSSI_MAX - $RSSI_MIN) + 1e-9);
        my $norm_snr  = ($snr->[$d][$g]  - $SNR_MIN)  / (($SNR_MAX  - $SNR_MIN)  + 1e-9);
        $norm_rssi = 0 if $norm_rssi < 0; $norm_rssi = 1 if $norm_rssi > 1;
        $norm_snr  = 0 if $norm_snr  < 0; $norm_snr  = 1 if $norm_snr  > 1;

        my $norm_load = ($OVERLOAD_THRESH > 0)
                      ? ($gw_load[$g] / $OVERLOAD_THRESH) : 0;
        $norm_load = 1 if $norm_load > 1; $norm_load = 0 if $norm_load < 0;

        # Calculate per-link interference (approximate)
        my $per_link_interf = 0;
        if ($chan_used) {
            my $current_chan = $chan_used->[$d][$g];
            for my $other_d (0 .. $NUM_DEV-1) {
                next if $other_d == $d;
                if ($gw_choice[$other_d] == $g && 
                    $chan_used->[$other_d][$g] == $current_chan) {
                    $per_link_interf += 0.1;
                }
            }
            $per_link_interf = min(1, $per_link_interf);
        } else {
            $per_link_interf = $interf;
        }

        # Calculate success probability
        my $sf = $current_sf{$d} || $DR_TO_SF{$SCENARIOS{$OPTIONS{scenario}}{default_dr}};
        my $p_succ = transmission_success_probability(
            $rssi->[$d][$g], $sf, $per_link_interf
        );

        $fit += $weights{ALPHA} * $norm_rssi
              + $weights{GAMMA} * $norm_snr
              - $weights{BETA}  * $norm_load
              - $weights{DELTA} * $per_link_interf
              + $LAMBDA_P * $p_succ;
    }
    
    return $fit - $fairness_penalty;
}

###############################  Optimization Algorithms  ####################
sub coati_assign {
    my ($rssi,$snr,$interf,$chan_used) = @_;

    # init population in [0,1]
    my @pop = map { [ map { rand() } 1 .. ($NUM_DEV * $NUM_GW) ] } 1 .. $POP_SIZE;
    my $best_fitness = -9e9;
    my $stagnation_count = 0;

    for my $t (1 .. $COATI_ITERS){
        my @fit = map { fitness($_, $rssi, $snr, undef, $interf, $chan_used) } @pop;
        my @sorted_idx = sort { $fit[$b] <=> $fit[$a] } 0 .. $#fit;
        
        # Check for stagnation
        if ($fit[$sorted_idx[0]] > $best_fitness) {
            $best_fitness = $fit[$sorted_idx[0]];
            $stagnation_count = 0;
        } else {
            $stagnation_count++;
        }
        
        # Stagnation restart
        if ($stagnation_count >= $STAGNATION_THRESHOLD) {
            my $num_to_reseed = int($POP_SIZE * $RESTART_RATE);
            for my $i (1 .. $num_to_reseed) {
                my $idx = $sorted_idx[-$i];
                $pop[$idx] = [ map { rand() } 1 .. ($NUM_DEV * $NUM_GW) ];
            }
            $stagnation_count = 0;
            @fit = map { fitness($_, $rssi, $snr, undef, $interf, $chan_used) } @pop;
            @sorted_idx = sort { $fit[$b] <=> $fit[$a] } 0 .. $#fit;
        }
        
        # Select top-k leaders
        my $k = min(3, $POP_SIZE);
        my @leaders = @pop[@sorted_idx[0..$k-1]];
        
        # Annealed noise
        my $eta0 = 0.2;
        my $eta = $eta0 * (1 - $t/$COATI_ITERS);
        
        for my $i (0 .. $#pop){
            next if $i == $sorted_idx[0];
            my $leader = $leaders[int(rand($k))];
            
            for my $j (0 .. $#{$pop[$i]}){
                my $step = (rand() < 0.5)
                         ? (rand() * $eta - $eta/2)
                         : ($leader->[$j] - $pop[$i][$j]) * rand();
                my $v = $pop[$i][$j] + $step;
                $v = 0 if $v < 0; $v = 1 if $v > 1;
                $pop[$i][$j] = $v;
            }
        }
    }

    # pick best vector by fitness
    my @fit = map { fitness($_, $rssi, $snr, undef, $interf, $chan_used) } @pop;
    my $best_idx = (sort { $fit[$b] <=> $fit[$a] } 0 .. $#fit)[0];
    my $best_vec = $pop[$best_idx];

    # ---- ARGMAX decode ----
    my @assign; my @gw_load = (0) x $NUM_GW;
    for my $d (0 .. $NUM_DEV-1){
        my $base = $d * $NUM_GW;
        my ($sel_gw,$bestv) = (0, -9e9);
        for my $g (0 .. $NUM_GW-1){
            my $v = $best_vec->[$base + $g];
            if ($v > $bestv){ $bestv = $v; $sel_gw = $g; }
        }
        push @assign, { device => $d, gateway => $sel_gw };
        $gw_load[$sel_gw]++;
    }
    return \@assign;
}
# --- NEW: compute fairness (Jain's index) from device_results.csv ---
sub compute_fairness_per_alg {
    my ($device_csv_path) = @_;
    my %fairness_by_alg;  # alg -> [ fairness values across iterations ]
    
    my $csv = Text::CSV->new({binary => 1});
    open my $dfh, '<', $device_csv_path or return %fairness_by_alg; # no device file? return empty
    
    # Header: Iteration,Algorithm,Device,Gateway, ...
    my $header = $csv->getline($dfh);
    my %col;
    for my $i (0..$#$header) { $col{$header->[$i]} = $i; }
    
    my %counts; # counts{Algorithm}{Iteration}{Gateway} = num_devices
    while (my $row = $csv->getline($dfh)) {
        my $alg  = $row->[$col{Algorithm}];
        my $iter = $row->[$col{Iteration}];
        my $gw   = $row->[$col{Gateway}];
        $counts{$alg}{$iter}{$gw}++;
    }
    close $dfh;
    
    # Jain's index J = (sum x_i)^2 / (n * sum x_i^2), where x_i = load on gateway i
    foreach my $alg (keys %counts) {
        foreach my $iter (keys %{$counts{$alg}}) {
            my @loads = values %{$counts{$alg}{$iter}};
            next unless @loads;
            my $n = scalar @loads;
            my $sum = 0; my $sum2 = 0;
            $sum  += $_ for @loads;
            $sum2 += ($_*$_) for @loads;
            my $jain = ($n > 0 && $sum2 > 0) ? ($sum*$sum) / ($n * $sum2) : 0;
            push @{$fairness_by_alg{$alg}}, $jain;
        }
    }
    return %fairness_by_alg;
}

sub improved_coati_assign {
    my ($rssi,$snr,$interf,$chan_used,$p_mut) = @_;
    $p_mut //= 0.10;

    my @pop = map { [ map { rand() } 1 .. ($NUM_DEV * $NUM_GW) ] } 1 .. $POP_SIZE;
    my $elite = $pop[0];
    my $best_fitness = -9e9;
    my $stagnation_count = 0;

    for my $t (1 .. $COATI_ITERS){
        my @fit = map { fitness($_, $rssi, $snr, undef, $interf, $chan_used) } @pop;
        my @sorted_idx = sort { $fit[$b] <=> $fit[$a] } 0 .. $#fit;
        
        # Check for stagnation
        if ($fit[$sorted_idx[0]] > $best_fitness) {
            $best_fitness = $fit[$sorted_idx[0]];
            $stagnation_count = 0;
        } else {
            $stagnation_count++;
        }
        
        # Stagnation restart
        if ($stagnation_count >= $STAGNATION_THRESHOLD) {
            my $num_to_reseed = int($POP_SIZE * $RESTART_RATE);
            for my $i (1 .. $num_to_reseed) {
                my $idx = $sorted_idx[-$i];
                $pop[$idx] = [ map { rand() } 1 .. ($NUM_DEV * $NUM_GW) ];
            }
            $stagnation_count = 0;
            @fit = map { fitness($_, $rssi, $snr, undef, $interf, $chan_used) } @pop;
            @sorted_idx = sort { $fit[$b] <=> $fit[$a] } 0 .. $#fit;
        }
        
        $elite = [ @{$pop[$sorted_idx[0]]} ];
        my $k = min(3, $POP_SIZE);
        my @leaders = @pop[@sorted_idx[0..$k-1]];
        
        # Annealed noise
        my $eta0 = 0.2;
        my $eta = $eta0 * (1 - $t/$COATI_ITERS);

        for my $i (0 .. $#pop){
            next if $i == $sorted_idx[0];
            my $leader = $leaders[int(rand($k))];
            
            for my $j (0 .. $#{$pop[$i]}){
                # pull towards leader with annealed noise
                my $v = $pop[$i][$j] + rand() * ($leader->[$j] - $pop[$i][$j])
                         + (rand() * $eta - $eta/2);
                
                # mutation
                if (rand() < $p_mut){
                    $v += (rand() * 0.5 - 0.25);
                }
                $v = 0 if $v < 0; $v = 1 if $v > 1;
                $pop[$i][$j] = $v;
            }
        }
        
        # elitism: replace current worst with previous elite
        $pop[$sorted_idx[-1]] = $elite;
    }

    my $best_vec = $elite;

    # ---- ARGMAX decode ----
    my @assign; my @gw_load = (0) x $NUM_GW;
    for my $d (0 .. $NUM_DEV-1){
        my $base = $d * $NUM_GW;
        my ($sel_gw,$bestv) = (0, -9e9);
        for my $g (0 .. $NUM_GW-1){
            my $v = $best_vec->[$base + $g];
            if ($v > $bestv){ $bestv = $v; $sel_gw = $g; }
        }
        push @assign, { device => $d, gateway => $sel_gw };
        $gw_load[$sel_gw]++;
    }
    return \@assign;
}

sub _repair_onehot {
    my ($vec) = @_;
    for my $d (0 .. $NUM_DEV-1){
        my $base = $d * $NUM_GW;
        # find argmax and set it 1, others 0
        my ($sel_gw,$bestv) = (0, -9e9);
        for my $g (0 .. $NUM_GW-1){
            my $v = $vec->[$base + $g];
            if ($v > $bestv){ $bestv = $v; $sel_gw = $g; }
        }
        for my $g (0 .. $NUM_GW-1){
            $vec->[$base + $g] = ($g == $sel_gw) ? 1 : 0;
        }
    }
}

sub binary_coati_assign {
    my ($rssi,$snr,$interf,$chan_used) = @_;

    # init binary populations (one 1 per device)
    my @pop;
    for (1 .. $POP_SIZE){
        my @v = (0) x ($NUM_DEV * $NUM_GW);
        for my $d (0 .. $NUM_DEV-1){
            my $g = int(rand($NUM_GW));
            $v[$d*$NUM_GW + $g] = 1;
        }
        push @pop, \@v;
    }

    my $best_fitness = -9e9;
    my $stagnation_count = 0;

    for my $t (1 .. $COATI_ITERS){
        my @fit = map { fitness($_, $rssi, $snr, undef, $interf, $chan_used) } @pop;
        my @sorted_idx = sort { $fit[$b] <=> $fit[$a] } 0 .. $#fit;
        
        # Check for stagnation
        if ($fit[$sorted_idx[0]] > $best_fitness) {
            $best_fitness = $fit[$sorted_idx[0]];
            $stagnation_count = 0;
        } else {
            $stagnation_count++;
        }
        
        # Stagnation restart
        if ($stagnation_count >= $STAGNATION_THRESHOLD) {
            my $num_to_reseed = int($POP_SIZE * $RESTART_RATE);
            for my $i (1 .. $num_to_reseed) {
                my $idx = $sorted_idx[-$i];
                my @v = (0) x ($NUM_DEV * $NUM_GW);
                for my $d (0 .. $NUM_DEV-1){
                    my $g = int(rand($NUM_GW));
                    $v[$d*$NUM_GW + $g] = 1;
                }
                $pop[$idx] = \@v;
            }
            $stagnation_count = 0;
            @fit = map { fitness($_, $rssi, $snr, undef, $interf, $chan_used) } @pop;
            @sorted_idx = sort { $fit[$b] <=> $fit[$a] } 0 .. $#fit;
        }
        
        my $leader = $pop[$sorted_idx[0]];
        my $k = min(3, $POP_SIZE);
        my @leaders = @pop[@sorted_idx[0..$k-1]];

        for my $i (0 .. $#pop){
            next if $i == $sorted_idx[0];
            my $current_leader = $leaders[int(rand($k))];
            
            for my $j (0 .. $#{$pop[$i]}){
                if (rand() < 0.70){
                    # copy leader bit
                    $pop[$i][$j] = $current_leader->[$j];
                } else {
                    # flip with small prob
                    if (rand() < 0.15){
                        $pop[$i][$j] = 1 - $pop[$i][$j];
                    }
                }
            }
            _repair_onehot($pop[$i]);
        }
    }

    # pick leader and decode (argmax; already one-hot)
    my @fit = map { fitness($_, $rssi, $snr, undef, $interf, $chan_used) } @pop;
    my $best_idx = (sort { $fit[$b] <=> $fit[$a] } 0 .. $#fit)[0];
    my $best_vec = $pop[$best_idx];

    my @assign;
    for my $d (0 .. $NUM_DEV-1){
        my $base = $d * $NUM_GW;
        my ($sel_gw,$bestv) = (0, -9e9);
        for my $g (0 .. $NUM_GW-1){
            my $v = $best_vec->[$base + $g];
            if ($v > $bestv){ $bestv = $v; $sel_gw = $g; }
        }
        push @assign, { device => $d, gateway => $sel_gw };
    }
    return \@assign;
}

sub ga_assign {
    my ($rssi,$snr,$interf,$chan_used) = @_;
    my @pop = map { [ map { int(rand($NUM_GW)) } 1..$NUM_DEV ] } 1..$GA_POP_SIZE;
    for (1..$GA_ITERS) {
        my @fit = map { ga_fitness($_,$rssi,$snr,$interf,$chan_used) } @pop;
        my @sorted = sort { $fit[$b] <=> $fit[$a] } 0..$#fit;
        my @offspring = ();
        for(my $i=0; $i<$GA_POP_SIZE/2; $i++){
            my($p1,$p2) = ($pop[$sorted[$i]],$pop[$sorted[$i+1]]);
            my $cut = int(rand($NUM_DEV));
            my @child = (@{$p1}[0..$cut-1], @{$p2}[$cut..$NUM_DEV-1]);
            push @offspring, \@child;
            @child = (@{$p2}[0..$cut-1], @{$p1}[$cut..$NUM_DEV-1]);
            push @offspring, \@child;
        }
        for my $c (@offspring){
            $c->[int(rand($NUM_DEV))] = int(rand($NUM_GW)) if rand() < $GA_MUT;
        }
        @pop = (@pop[@sorted[0..1]], @offspring[0..$GA_POP_SIZE-3]);
    }
    my $bestpop = (sort { ga_fitness($b,$rssi,$snr,$interf)<=>ga_fitness($a,$rssi,$snr,$interf) } @pop)[0];
    my @assign = map { {device=>$_,gateway=>$bestpop->[$_]} } 0..$NUM_DEV-1;
    return \@assign;
}

sub pso_assign {
    my ($rssi,$snr,$interf,$chan_used)=@_;
    my @pop = map { [ map { rand($NUM_GW) } 1..$NUM_DEV ] } 1..$PSO_POP_SIZE;
    my @vel = map { [ map { 0 } 1..$NUM_DEV ] } 1..$PSO_POP_SIZE;
    my @pbests = map { [ @{$pop[$_]} ] } 0..$#pop;
    my @pbestval = map { pso_fitness($pop[$_],$rssi,$snr,$interf,$chan_used) } 0..$#pop;
    my $gbest = [ @{$pop[0]} ]; my $gbestval = $pbestval[0];

    for (1..$PSO_ITERS){
        for my $i(0..$#pop){
            for my $j(0..$NUM_DEV-1){
                $vel[$i][$j] = 0.5*$vel[$i][$j] + 1.5*rand()*($pbests[$i][$j]-$pop[$i][$j]) + 1.5*rand()*($gbest->[$j]-$pop[$i][$j]);
                $pop[$i][$j] += $vel[$i][$j];
                $pop[$i][$j] = 0 if $pop[$i][$j]<0;
                $pop[$i][$j] = $NUM_GW-1 if $pop[$i][$j]>$NUM_GW-1;
            }
            my @iant = map { int($_+0.5) } @{$pop[$i]};
            my $fit = pso_fitness(\@iant,$rssi,$snr,$interf,$chan_used);
            if($fit > $pbestval[$i]){ $pbests[$i]=[@iant]; $pbestval[$i]=$fit; }
            if($fit > $gbestval)    { $gbest=[@iant]; $gbestval=$fit; }
        }
    }
    my @assign = map { {device=>$_, gateway=>$gbest->[$_]} } 0..$NUM_DEV-1;
    return \@assign;
}

sub alg_hr_assignment { 
    my($d,$rssi)=@_; 
    my @idx= sort{$rssi->[$d][$b]<=>$rssi->[$d][$a]}0..$NUM_GW-1;
    return $idx[0]; 
}

# sub alg_lb_assignment { 
#     my($d, $load) = @_; 
#     my @idx = sort {$load->[$a] <=> $load->[$b]} 0..$NUM_GW-1; 
#     return $idx[0]; 
# }
sub alg_lb_assignment {
    my ($d,$load)=@_;

    my $min_load = min(@$load);

    my @candidate;

    for my $g (0..$NUM_GW-1){
        push @candidate,$g if $load->[$g]==$min_load;
    }

    return $candidate[int(rand(@candidate))];
}
sub e_alg_lbhr_assignment {
    my ($d, $rssi, $snr, $load) = @_;

    my ($best, $best_sc) = (-1, -9e9);

    for my $g (0..$NUM_GW-1) {

        # Normalize RSSI to [0,1]
        my $norm_rssi = ($rssi->[$d][$g] + 120) / 40;
        $norm_rssi = 0 if $norm_rssi < 0;
        $norm_rssi = 1 if $norm_rssi > 1;

        # Normalize SNR to [0,1]
        my $norm_snr = ($snr->[$d][$g] + 20) / 30;
        $norm_snr = 0 if $norm_snr < 0;
        $norm_snr = 1 if $norm_snr > 1;

        # Normalize gateway load to [0,1]
        my $norm_load = ($OVERLOAD_THRESH > 0)
            ? ($load->[$g] / $OVERLOAD_THRESH)
            : 0;
        $norm_load = 0 if $norm_load < 0;
        $norm_load = 1 if $norm_load > 1;

        # Weighted score
        my $score =
              $weights{ALPHA} * $norm_rssi
            + $weights{GAMMA} * $norm_snr
            - $weights{BETA}  * $norm_load;

        if ($score > $best_sc) {
            $best    = $g;
            $best_sc = $score;
        }
    }

    return $best;
}
 sub alg_lbhr_assignment {
    my ($d, $rssi, $load) = @_;

    # Step 1: Find minimum gateway load
    my $min_load = min(@$load);

    # Step 2: Candidate gateways having minimum load
    my @candidate;
    for my $g (0..$NUM_GW-1) {
        push @candidate, $g if $load->[$g] == $min_load;
    }

    # Step 3: Choose highest RSSI among candidates
    my $best = $candidate[0];
    foreach my $g (@candidate) {
        if ($rssi->[$d][$g] > $rssi->[$d][$best]) {
            $best = $g;
        }
    }

    return $best;
}

###############################  Data Generation  ###########################
sub gen_random_data {
    my @rssi;       # [d][g]
    my @chan_used;  # [d][g]
    for my $d (0 .. $NUM_DEV-1) {
        my (@row_rssi, @row_ch);
        for my $g (0 .. $NUM_GW-1) {
            my $chan = $CHANNELS[int(rand(@CHANNELS))];
            my $dist = rand(100) + 10;
            my $txp  = $current_power{$d} // $DEFAULT_POWER;
            push @row_rssi, calculate_rssi($dist, $chan, $txp);
            push @row_ch,   $chan;
        }
        push @rssi,      \@row_rssi;
        push @chan_used, \@row_ch;
    }

    my @snr;  # per-link SNR derived consistently from RSSI
    for my $d (0 .. $NUM_DEV-1){
        my @row;
        for my $g (0 .. $NUM_GW-1){
            push @row, calculate_snr($rssi[$d][$g]);  # respect noise & scenario interferers inside
        }
        push @snr, \@row;
    }

    my @traffic = map { int(rand($MAX_PAYLOAD)) + 1 } (0 .. $NUM_DEV-1);
    return (\@rssi, \@snr, \@traffic, \@chan_used);
}

###############################  Initialization  ############################
sub initialize_devices {
    for my $d (0..$NUM_DEV-1) {
        $current_sf{$d} = $DR_TO_SF{$SCENARIOS{$OPTIONS{scenario}}{default_dr}};
        $current_power{$d} = $DEFAULT_POWER;
        for my $g (0..$NUM_GW-1) {
            $rssi_history{$d}{$g} = [];
        }
    }
}

###############################  Enhanced Simulation  ########################
sub sim_tx {
    my ($assign, $rssi, $traffic, $snr, $interf, $chan_used, $current_time) = @_;

    my $NUM_CH = scalar(@CHANNELS);
    $NUM_CH = 1 if $NUM_CH < 1;

    my @gw_load = (0) x $NUM_GW;
    my %gw_ch_trans;
    my %res;

    # Initialize device states
    for my $d (0..$NUM_DEV-1) {
        enter_state($d, 'IDLE', $current_time);
    }

    # Reset duty cycle windows at the beginning of each simulation step
    reset_duty_cycle_windows($current_time);

    # bucket by (gateway, channel)
    foreach my $a (@$assign) {
        my $d = $a->{device};
        my $g = $a->{gateway};
        my $ch = (defined $chan_used && defined $chan_used->[$d][$g]) ? 
                 $chan_used->[$d][$g] : $CHANNELS[0];
        push @{ $gw_ch_trans{$g}{$ch} }, $d;
    }

    # Track UL events for ACK latency calculation
    my %ul_events_this_iter;
    
    foreach my $a (@$assign) {
        my $d  = $a->{device};
        my $g  = $a->{gateway};
        my $ch = (defined $chan_used && defined $chan_used->[$d][$g]) ? 
                 $chan_used->[$d][$g] : $CHANNELS[0];

        my $sf    = exists $current_sf{$d}    ? $current_sf{$d}    : 7;
        my $power = exists $current_power{$d} ? $current_power{$d} : 14;

        my $pkt_sent  = segment_traffic($traffic->[$d], $MAX_PAYLOAD);
        my $pkt_atime = airtime($sf, 125000, $MAX_PAYLOAD);
        $airtimes{$d} = $pkt_sent * $pkt_atime;

        # Update device state to TX
        enter_state($d, 'TX', $current_time);
        my $tx_end_time = $current_time + $pkt_atime * $pkt_sent;

        # Register UL event for ACK latency calculation
        $ul_events_this_iter{$d} = {
            start_time => $current_time,
            end_time => $tx_end_time,
            gw_id => $g,
            sf => $sf,
            dr => $SF_TO_DR{$sf},
            confirmed => (rand() < $OPTIONS{confirmed}) ? 1 : 0
        };

        # Register signal for collision detection
        my $signal_id = "dev_${d}_tx";
        register_signal($g, $signal_id, $current_time, $tx_end_time, 
                       $rssi->[$d][$g], $sf, $d);

        # ADR sampling
        push @{ $rssi_history{$d}{$g} }, $rssi->[$d][$g];
        my $history_size = scalar @{ $rssi_history{$d}{$g} // [] };
        if ($history_size >= 5 && ($history_size % 5) == 0) {
            my ($new_sf, $new_power) = adapt_data_rate($d, $g, $sf, $power, $rssi_history{$d}{$g});
            if ($new_sf != $sf || $new_power != $power) {
                $adr_updates{$d} = [$new_sf, $new_power];
            }
        }

        # Check for collisions
        my @overlapping = resolve_overlaps($g, $tx_end_time);
        my $collision = @overlapping > 1;
        my $capture_win = 0;

        if ($collision) {
            # Find the strongest signal
            my @sorted = sort { $b->{power} <=> $a->{power} } @overlapping;
            my $strongest = $sorted[0];
            
            # Check if our signal is the strongest
            if ($strongest->{dev} == $d) {
                $capture_win = 1;
                
                # Check capture against each weaker signal
                for my $i (1..$#sorted) {
                    my $weaker = $sorted[$i];
                    my $power_diff = $strongest->{power} - $weaker->{power};
                    my $threshold = $CAPTURE_THRESHOLDS[$strongest->{sf}-7][$weaker->{sf}-7];
                    
                    if ($power_diff < $threshold) {
                        $capture_win = 0;
                        last;
                    }
                }
            }
        }

        # Per-channel collision factor
        my $load_on_ch = scalar(@{ $gw_ch_trans{$g}{$ch} // [] });
        my $cap_per_ch = ($OVERLOAD_THRESH > 0) ? ($OVERLOAD_THRESH / $NUM_CH) : 1;
        my $collision_factor = $load_on_ch / ($cap_per_ch > 0 ? $cap_per_ch : 1);
        $collision_factor = 1 if $collision_factor > 1; 
        $collision_factor = 0 if $collision_factor < 0;

        # Apply ADR updates
        if (exists $adr_updates{$d}) {
            ($current_sf{$d}, $current_power{$d}) = @{ $adr_updates{$d} };
            delete $adr_updates{$d};
            $sf    = $current_sf{$d};
            $power = $current_power{$d};
        }

        # Attempt + retries with EffPDR calculation
        my ($recv, $retry) = (0, 0);
        for my $i (1 .. $pkt_sent) {
            my $ok = 0;
            
            if ($collision) {
                $ok = $capture_win ? 1 : 0;
            } else {
                my $eff_interf = $collision_factor + $interf;
                $ok = transmission_success($rssi->[$d][$g], $sf, $eff_interf);
            }
            
            if ($ok) {
                $recv++;
            } else {
                for my $j (1 .. $MAX_RETRIES) {
                    my $eff_interf = $collision_factor + $interf;
                    if (transmission_success($rssi->[$d][$g], $sf, $eff_interf)) { 
                        $recv++; 
                        last; 
                    } else { 
                        $retry++; 
                    }
                }
            }
        }

        # Calculate energy and latency
        my $dist   = rand(100) + 10;
        my $energy = calculate_energy_consumption($d, $tx_end_time);
        my $lat    = $BASE_LATENCY + $collision_factor * $LOAD_LAT_FACTOR + $dist * $DIST_LAT_FACTOR;

        $gw_load[$g] += $pkt_sent;

        # Return to idle state after transmission
        enter_state($d, 'IDLE', $tx_end_time);

        $res{$d} = {
            gateway => $g,
            channel => $ch,
            rssi    => $rssi->[$d][$g],
            snr     => $snr->[$d][$g],
            sf      => $sf,
            power   => $power,
            traffic => $traffic->[$d],
            sent    => $pkt_sent,
            recv    => $recv,
            energy  => $energy,
            lat     => $lat,
            over    => ($gw_load[$g] > $OVERLOAD_THRESH ? 'YES' : 'NO'),
            collision => $collision ? 'YES' : 'NO',
            capture_win => $capture_win ? 'YES' : 'NO',
            retries => $retry
        };

        # Update ADR-ACK counter
        adr_ack_update($d, 0);  # Assume no downlink for now
        
        # Track retransmission count
        $retx_count{$d} = ($retx_count{$d} || 0) + $retry;
    }

    # Process ACKs for confirmed uplinks
    process_acks(\%ul_events_this_iter, $current_time);
    
    return %res;
}

sub process_acks {
    my ($ul_events, $current_time) = @_;
    
    foreach my $dev_id (keys %$ul_events) {
        my $ul_event = $ul_events->{$dev_id};
        
        # Only process confirmed uplinks
        next unless $ul_event->{confirmed};
        
        # Select gateway for ACK
        my $ack_info = select_gw_for_ack(
            $dev_id, 
            $ul_event->{end_time}, 
            $ul_event->{dr}, 
            0  # RX1 offset
        );
        
        next unless $ack_info;  # Skip if no gateway found
        
        # Calculate ACK airtime
        my $ack_airtime = airtime($ul_event->{sf}, 125000, 0);  # 0-byte ACK
        
        # Schedule ACK transmission
        my $ack_schedule = schedule_ack_tx(
            $ack_info->{gw},
            $CHANNELS[0],  # Default channel for ACK
            $ack_info->{window},
            $ack_airtime
        );
        
        if ($ack_schedule) {
            # Record ACK event
            $ack_events{$dev_id} = {
                ul_end_time => $ul_event->{end_time},
                ack_start_time => $ack_schedule->{start},
                ack_end_time => $ack_schedule->{end},
                gw_id => $ack_info->{gw},
                window_type => $ack_info->{type},
                success => 1
            };
            
            # Update ADR-ACK counter
            adr_ack_update($dev_id, 1);
        } else {
            # ACK scheduling failed
            $ack_events{$dev_id} = {
                ul_end_time => $ul_event->{end_time},
                success => 0
            };
        }
    }
}

sub calc_metrics {
    my($agg)=@_;
    my ($sent,$recv,$e_sum,$d_sum)=(0,0,0,0);
    my ($tot_bytes,$tot_time)=(0,0);
    my ($sf_sum,$power_sum)=(0,0);
    my ($eff_sent, $eff_recv) = (0, 0);
    
    while(my($d,$h)=each %$agg){
        $sent += $h->{sent};
        $recv += $h->{recv};
        $e_sum += $h->{energy};
        $d_sum += $h->{lat};
        $tot_bytes += $h->{traffic};
        $tot_time  += $h->{lat}/1000;
        $sf_sum += $h->{sf};
        $power_sum += $h->{power};
        
        # Calculate effective PDR (collision-aware)
        my $collision_penalty = $h->{collision} eq 'YES' ? 0.7 : 1.0;
        $eff_sent += $h->{sent};
        $eff_recv += $h->{recv} * $collision_penalty;
    }
    
    my $pdr   = $sent ? $recv/$sent : 0;
    my $eff_pdr = $eff_sent ? $eff_recv/$eff_sent : 0;
    my $pcr   = 1 - $pdr;
    my $delay = $recv ? $d_sum/$recv : 0;
    my $thr   = $tot_time ? $tot_bytes/$tot_time : 0;
    my $avg_sf = $NUM_DEV ? $sf_sum/$NUM_DEV : 0;
    my $avg_power = $NUM_DEV ? $power_sum/$NUM_DEV : 0;
    
    return ($pdr, $eff_pdr, $pcr, $e_sum, $delay, $thr, $avg_sf, $avg_power);
}

sub calculate_ack_rate {
    my ($alg) = @_;
    my $total_confirmed = 0;
    my $total_acks = 0;
    
    foreach my $dev_id (keys %ack_events) {
        $total_confirmed++;
        $total_acks++ if $ack_events{$dev_id}{success};
    }
    
    return $total_confirmed ? $total_acks / $total_confirmed : 0;
}

sub calculate_retx_rate {
    my ($alg) = @_;
    my $total_packets = 0;
    my $total_retx = 0;
    
    foreach my $dev_id (keys %retx_count) {
        $total_packets += ($retx_count{$dev_id} + 1);  # Original + retries
        $total_retx += $retx_count{$dev_id};
    }
    
    return $total_packets ? $total_retx / $total_packets : 0;
}

sub calculate_latency_stats {
    my ($alg) = @_;
    my @latencies;
    
    foreach my $dev_id (keys %ack_events) {
        next unless $ack_events{$dev_id}{success};
        my $latency = $ack_events{$dev_id}{ack_end_time} - $ack_events{$dev_id}{ul_end_time};
        push @latencies, $latency;
    }
    
    return (0, 0) unless @latencies;
    
    my $stat = Statistics::Descriptive::Full->new();
    $stat->add_data(@latencies);
    
    my $mean = $stat->mean();
    my $p95 = $stat->percentile(95);
    
    return ($mean, $p95);
}

###############################  Main Simulation Loop #######################
sub main {
    parse_cli();
    
    # Initialize devices and gateways
    for my $d (0..$NUM_DEV-1) {
        $current_sf{$d} = $DR_TO_SF{$SCENARIOS{$OPTIONS{scenario}}{default_dr}};
        $current_power{$d} = $DEFAULT_POWER;
        for my $g (0..$NUM_GW-1) {
            $rssi_history{$d}{$g} = [];
        }
    }
    
    for my $gw_id (0..$NUM_GW-1) {
        init_gw_duty($gw_id);
    }

    # Open CSV files for results
    my $csv_dev = Text::CSV->new({binary=>1, eol=>"\n"});
    open my $DEV_FH, '>', "device_results.csv" or die $!;
    $csv_dev->print($DEV_FH, [qw(Iteration Algorithm Device Gateway RSSI SNR SF Power Traffic Packets_Sent Packets_Recv Energy_mJ Latency_ms Overloaded Collision Capture Retries)]);

    open my $ALG_FH, '>', "algorithm_results.csv" or die $!;
   # print $ALG_FH "Algorithm,Iteration,PDR,EffPDR,Energy_mJ,Delay_ms,Throughput,AvgSF,AvgPower,WeightConfig,ACKRate,RetxRate,ULLatencyMean,ULLatencyP95\n";
    print $ALG_FH "Algorithm,Iteration,PDR,EffPDR,Energy_mJ,Delay_ms,Throughput,AvgSF,AvgPower,ACKRate,RetxRate,ULLatencyMean,ULLatencyP95,WeightConfig\n";
    my $current_time = 0;
    
    for my $iter (1..$OPTIONS{iters}) {
        print "--- Iteration $iter ---\n";
        my ($RSSI, $SNR, $TRAFFIC, $CHAN_USED) = gen_random_data();
        
        # Generate assignments using different algorithms
        my %assignments;
        
        # Alg-LB (load balancing only)
        my @assign_ALG_LB;
        my @gw_load = (0) x $NUM_GW;
        for my $d (0..$NUM_DEV-1) {
            my $gw = alg_lb_assignment($d, \@gw_load);
            push @assign_ALG_LB, { device => $d, gateway => $gw };
            $gw_load[$gw]++;
        }

        # Alg-LBHR (RSSI + load + SNR)
        my @assign_ALG_LBHR;
        @gw_load = (0) x $NUM_GW;
        for my $d (0..$NUM_DEV-1) {
            my $gw = alg_lbhr_assignment($d, $RSSI, \@gw_load);
            push @assign_ALG_LBHR, { device => $d, gateway => $gw };
            $gw_load[$gw]++;
        }
        # E-Alg-LBHR
my @assign_E_ALG_LBHR;
@gw_load = (0) x $NUM_GW;

for my $d (0..$NUM_DEV-1) {
    my $gw = e_alg_lbhr_assignment($d, $RSSI, $SNR, \@gw_load);

    push @assign_E_ALG_LBHR, {
        device  => $d,
        gateway => $gw
    };

    $gw_load[$gw]++;
}
        
       %assignments = (
    'CoATI'          => coati_assign($RSSI, $SNR, $OPTIONS{interf}, $CHAN_USED),
    'Improved-CoATI' => improved_coati_assign($RSSI, $SNR, $OPTIONS{interf}, $CHAN_USED),
    'Binary-CoATI'   => binary_coati_assign($RSSI, $SNR, $OPTIONS{interf}, $CHAN_USED),
    'GA'             => ga_assign($RSSI, $SNR, $OPTIONS{interf}, $CHAN_USED),
    'PSO'            => pso_assign($RSSI, $SNR, $OPTIONS{interf}, $CHAN_USED),
    'Alg-HR'         => [ map { {device=>$_, gateway=>alg_hr_assignment($_,$RSSI)} } 0..$NUM_DEV-1 ],
    'Alg-LB'         => \@assign_ALG_LB,
    'Alg-LBHR'       => \@assign_ALG_LBHR,
    'E-Alg-LBHR'     => \@assign_E_ALG_LBHR,
);
        
        my %metrics; 
        my $best_alg; 
        my $best_pdr = -1;

        foreach my $alg (keys %assignments) {
            #next if $OPTIONS{alg} ne 'all' && $OPTIONS{alg} ne $alg;
            if ($OPTIONS{alg} ne 'all') {
        my @selected_algs = split(',', $OPTIONS{alg});
        next unless grep { $_ eq $alg } @selected_algs;
    }
            # Reset ACK and retx tracking for each algorithm
            %ack_events = ();
            %retx_count = ();
            
            my %res = sim_tx($assignments{$alg}, $RSSI, $TRAFFIC, $SNR, 
                           $OPTIONS{interf}, $CHAN_USED, $current_time);
            my @metrics = calc_metrics(\%res);
            $metrics{$alg} = \@metrics;

            # Write device results
            foreach my $d (sort {$a<=>$b} keys %res) {
                my $h = $res{$d};
                $csv_dev->print($DEV_FH, [
                    $iter, $alg, $d, $h->{gateway}, $h->{rssi}, $h->{snr},
                    $h->{sf}, $h->{power}, $h->{traffic}, $h->{sent}, $h->{recv},
                    sprintf("%.3f",$h->{energy}), sprintf("%.2f",$h->{lat}), 
                    $h->{over}, $h->{collision}, $h->{capture_win}, $h->{retries}
                ]);
            }
            
            # Calculate additional metrics
            my $ack_rate = calculate_ack_rate($alg);
            my $retx_rate = calculate_retx_rate($alg);
            my ($ul_lat_mean, $ul_lat_p95) = calculate_latency_stats($alg);
            # Ensure default values if no ACK events occurred
	    $ul_lat_mean = defined $ul_lat_mean ? $ul_lat_mean : 0;
	    $ul_lat_p95 = defined $ul_lat_p95 ? $ul_lat_p95 : 0;
            # Write algorithm results
            my ($pdr, $eff_pdr, $pcr, $e, $delay, $thr, $avg_sf, $avg_power) = @metrics;
            my $weight_info = "ALPHA=$weights{ALPHA},BETA=$weights{BETA},GAMMA=$weights{GAMMA},DELTA=$weights{DELTA}";

print $ALG_FH join(',', (
    $alg, $iter, sprintf("%.4f", $pdr), sprintf("%.4f", $eff_pdr),
    sprintf("%.2f", $e), sprintf("%.2f", $delay), sprintf("%.2f", $thr),
    sprintf("%.2f", $avg_sf), sprintf("%.2f", $avg_power),
    sprintf("%.3f", $ack_rate), sprintf("%.3f", $retx_rate),
    sprintf("%.2f", $ul_lat_mean), sprintf("%.2f", $ul_lat_p95),
    $weight_info  # Move to the end to avoid interfering with numeric columns
)) . "\n";
            
            # Update best algorithm - FIXED: Check if best_alg is defined
            if (defined $best_alg) {
                if ($pdr > $best_pdr || 
                    ($pdr == $best_pdr && $e < $metrics{$best_alg}[3])) {
                    $best_alg = $alg; 
                    $best_pdr = $pdr;
                }
            } else {
                $best_alg = $alg;
                $best_pdr = $pdr;
            }
        }

        # FIXED: Check if best_alg is defined before printing
        if (defined $best_alg) {
            print "Best algorithm this iteration: $best_alg (PDR=", sprintf("%.3f", $best_pdr), ")\n";
        } else {
            print "No valid algorithms processed this iteration\n";
        }
        $current_time += 3600;
    }

    close $DEV_FH; 
    close $ALG_FH;
    print "\nSimulation complete. Results written to CSVs.\n";
    
    generate_summary_stats();
}
sub generate_summary_stats {
    # Read algorithm results and device-level fairness; write statistical summary
    my $csv = Text::CSV->new({binary => 1});
    open my $fh, '<', 'algorithm_results.csv' or die "Cannot open algorithm_results.csv: $!";
    
    my %results;  # results{alg}{metric} = [values...]
    my $header = <$fh>;  # Skip header
    
    while (my $row = $csv->getline($fh)) {
        my ($alg, $iter, $pdr, $eff_pdr, $energy, $delay, $thr, $avg_sf, $avg_power, 
            $ack_rate, $retx_rate, $ul_lat_mean, $ul_lat_p95, $weight_config) = @$row;
        
        push @{$results{$alg}{pdr}},            $pdr;
        push @{$results{$alg}{eff_pdr}},        $eff_pdr;
        push @{$results{$alg}{energy}},         $energy;
        push @{$results{$alg}{delay}},          $delay;
        push @{$results{$alg}{throughput}},     $thr;
        push @{$results{$alg}{avg_sf}},         $avg_sf;
        push @{$results{$alg}{avg_power}},      $avg_power;
        push @{$results{$alg}{ack_rate}},       $ack_rate;
        push @{$results{$alg}{retx_rate}},      $retx_rate;
        push @{$results{$alg}{ul_lat_mean}},    $ul_lat_mean;
        push @{$results{$alg}{ul_lat_p95}},     $ul_lat_p95;
    }
    close $fh;

    # --- NEW: inject fairness from device_results.csv ---
    my %fairness = compute_fairness_per_alg('device_results.csv');  # alg -> [values]
    foreach my $alg (keys %fairness) {
        $results{$alg}{fairness} = $fairness{$alg};
    }
    
    # Scenario metadata
    my $environment = $OPTIONS{scenario};
    my $devices = $NUM_DEV;
    my $gateways = $NUM_GW;
    my $ed_per_gw = $gateways > 0 ? sprintf("%.2f", $devices / $gateways) : "N/A";
    
    my ($dr_min, $dr_max);
    if ($environment eq 'Industrial') {
        ($dr_min, $dr_max) = ('DR3', 'DR5');
    } elsif ($environment eq 'Suburban') {
        ($dr_min, $dr_max) = ('DR2', 'DR4');
    } else { # Rural-Agricultural
        ($dr_min, $dr_max) = ('DR0', 'DR2');
    }

    my $interference_level;
    my $interf = $OPTIONS{interf};
    if ($interf <= 0.1)      { $interference_level = 'low'; }
    elsif ($interf <= 0.3)   { $interference_level = 'medium'; }
    elsif ($interf <= 0.5)   { $interference_level = 'high'; }
    else                     { $interference_level = 'very_high'; }
    
    my $traffic_type = $OPTIONS{traffic};
    my $confirmed_ratio = $OPTIONS{confirmed};
    my $weight_config = $OPTIONS{weight_config};
    my $seed = $OPTIONS{seed};
    my $iters = $OPTIONS{iters};
    
    open my $summary_fh, '>', 'summary_statistics.csv' or die "Cannot create summary_statistics.csv: $!";
    print $summary_fh join(',', (
        "Algorithm","Metric","Mean","StdDev","CI95_Low","CI95_High",
        "Environment","Devices","Gateways","ED_per_GW",
        "DR_Min","DR_Max","InterferenceLevel","TrafficType",
        "ConfirmedRatio","WeightConfig","Seed","Iters"
    )) . "\n";
    
    foreach my $alg (sort keys %results) {
        foreach my $metric (sort keys %{$results{$alg}}) {
            my @values = grep { looks_like_number($_) } @{$results{$alg}{$metric} // []};
            next unless @values;
            my $n = scalar @values;
            my $m = mean(\@values);
            my $sd = stddev(\@values);
            my $t = 1.96; # normal approx
            my $ci_low  = $m - $t * ($sd / sqrt($n));
            my $ci_high = $m + $t * ($sd / sqrt($n));
            
            print $summary_fh join(',', (
                $alg, $metric,
                sprintf("%.4f",$m), sprintf("%.4f",$sd),
                sprintf("%.4f",$ci_low), sprintf("%.4f",$ci_high),
                $environment, $devices, $gateways, $ed_per_gw,
                $dr_min, $dr_max, $interference_level, $traffic_type,
                $confirmed_ratio, $weight_config, $seed, $iters
            )) . "\n";
        }
    }
    close $summary_fh;
    print "Summary statistics written to summary_statistics.csv\n";
}

#sub generate_summary_stats {
    # Read algorithm results and generate statistical summary
 #   my $csv = Text::CSV->new({binary => 1});
  #  open my $fh, '<', 'algorithm_results.csv' or die "Cannot open algorithm_results.csv: $!";
    
   # my %results;
   # my $header = <$fh>;  # Skip header
    
    #while (my $row = $csv->getline($fh)) {
    # Skip the WeightConfig column (last column, position 14)
    #my ($alg, $iter, $pdr, $eff_pdr, $energy, $delay, $thr, $avg_sf, $avg_power, $ack_rate, $retx_rate, $ul_lat_mean, $ul_lat_p95, $weight_config) = @$row;
    
    # Only process numeric columns for statistics
    #push @{$results{$alg}{pdr}}, $pdr;
    #push @{$results{$alg}{eff_pdr}}, $eff_pdr;
   # push @{$results{$alg}{energy}}, $energy;
    #push @{$results{$alg}{delay}}, $delay;
   # push @{$results{$alg}{throughput}}, $thr;
    #push @{$results{$alg}{avg_sf}}, $avg_sf;
   # push @{$results{$alg}{avg_power}}, $avg_power;
    #push @{$results{$alg}{ack_rate}}, $ack_rate;
   # push @{$results{$alg}{retx_rate}}, $retx_rate;
    #push @{$results{$alg}{ul_lat_mean}}, $ul_lat_mean;
   # push @{$results{$alg}{ul_lat_p95}}, $ul_lat_p95;
    # Skip $weight_config as it's not numeric
#}
 #   close $fh;
    
    # Generate summary statistics
  #  open my $summary_fh, '>', 'summary_statistics.csv' or die "Cannot create summary_statistics.csv: $!";
   # print $summary_fh "Algorithm,Metric,Mean,StdDev,CI95_Low,CI95_High\n";
    
    #foreach my $alg (keys %results) {
     #   foreach my $metric (keys %{$results{$alg}}) {
      #      my @values = @{$results{$alg}{$metric}};
       #     my $mean = mean(\@values);
        #    my $stddev = stddev(\@values);
         #   my $n = scalar @values;
          #  my $t_value = 1.96;  # For 95% CI with large n
           # my $ci_low = $mean - $t_value * ($stddev / sqrt($n));
           # my $ci_high = $mean + $t_value * ($stddev / sqrt($n));
            
            #print $summary_fh join(',', (
             #   $alg, $metric, 
              #  sprintf("%.4f", $mean), 
               # sprintf("%.4f", $stddev),
                #sprintf("%.4f", $ci_low),
               # sprintf("%.4f", $ci_high)
           # )) . "\n";
     #   }
    #}
    #close $summary_fh;
    
    #print "Summary statistics written to summary_statistics.csv\n";
#}

# Run the main simulation
main();
