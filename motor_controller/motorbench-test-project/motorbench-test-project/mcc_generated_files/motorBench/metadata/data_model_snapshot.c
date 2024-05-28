/*
 * data_model_snapshot.c
 * 
 * Snapshot of code generation data model.
 * This information is included in the executable file
 * and does not affect any memory content in the embedded device.
 */
 
__attribute__((section("diagnostics_cinfo, info"), keep)) char MCAF_data_model_snapshot[] =
  "{\"motor\": {\"Tfr\": 0.0048125,\"B\": 3.22134e-05,\"Lq\": 0.000349495,\"poleCou"
  "nt\": 10,\"rated_current\": 4.53,\"J\": 1.80792e-05,\"velocity\": {\"nominal\": "
  "381.5987877,\"maximum\": 628.3185307,\"units\": \"rad\\/s\"},\"L\": 0.000358981,"
  "\"R\": 0.37160175,\"current\": {\"maximum\": {\"continuous\": 4.53,\"peak\": 4.5"
  "3},\"units\": \"A\"},\"Ld\": 0.000368467,\"Ke\": 0.0371468869868,\"id\": \"Hurst"
  "300\"},\"controller\": {\"encoder\": {\"tracking_loop\": {\"zeta\": 1.5}},\"foc"
  "\": {\"feedback\": {\"voltage_delay\": 0}},\"pll\": {\"delay_match\": true,\"pro"
  "perties\": {\"reference\": false},\"keInverseScaling\": 1.0},\"operation\": {\"m"
  "in_deceleration_time\": null},\"dyn_current\": {\"maxgain\": 1.1},\"zsmt\": {\"e"
  "rrorLimit\": 0.99997,\"properties\": {\"independent\": true,\"reference\": false"
  "},\"duration\": {\"lockDelay\": 0.05},\"probe\": {\"accumGain\": 1.0}},\"fault_d"
  "etect\": {\"recovery\": {\"attempts\": 3}}},\"metadata\": {\"MCC\": {\"architect"
  "ure\": \"melody\",\"peripherals\": {\"instances\": [{\"name\": \"ADC1\",\"type\""
  ": \"adc\"},{\"name\": \"PWM\",\"type\": \"pwm_hs\"},{\"name\": \"QEI1\",\"type\""
  ": \"qei\"},{\"name\": \"TMR1\",\"type\": \"timer\"},{\"name\": \"SCCP1\",\"type"
  "\": \"timer\"},{\"name\": \"UART1\",\"type\": \"uart\"},{\"name\": \"OPA1\",\"ty"
  "pe\": \"opa\"},{\"name\": \"OPA2\",\"type\": \"opa\"},{\"name\": \"OPA3\",\"type"
  "\": \"opa\"}]},\"version\": \"5.7.0\",\"peripheral_library\": {\"name\": \"melod"
  "y\",\"version\": \"NA\"}},\"data_model\": {\"schema_version\": 5},\"motorBench\""
  ": {\"commit_id\": \"6062ee969f17e173cb8502638f1e3c8ee7bea3fb\",\"version\": \"2."
  "45.0\"},\"MCAF\": {\"build\": {\"commit_id\": \"116330\",\"date_str\": \"2023 Fe"
  "b 09\",\"hostname\": \"motor-control-mcaf-tags-builder-r7-2frc37-1-0ws3m-smcgr-k"
  "thrw\",\"time\": 1675959683.33,\"time_iso\": \"2023-02-09T16:21:23.327489\",\"ti"
  "me_str\": \"2023 Feb 09 16:21\",\"type\": \"release\",\"username\": \"jenkins\","
  "\"version\": \"R7\\/RC37\"},\"build_time\": 1675959683.33,\"build_time_iso\": \""
  "2023-02-09T16:21:23.327489\",\"build_time_str\": \"2023 Feb 09 16:21\",\"commit_"
  "id\": \"116330\",\"version\": \"R7\\/RC37\"}},\"scdata\": {\"setup\": {\"communi"
  "cations\": {\"baudrate\": 625000},\"analysis\": {\"spindown_time\": {\"units\": "
  "\"s\",\"value\": 1.0},\"max_motor_speed\": {\"units\": \"rad\\/s mechanical\",\""
  "value\": 2094.4}},\"calibration\": {\"vdc\": {\"units\": \"V\",\"value\": 24.0},"
  "\"resistor\": {\"units\": \"ohms\",\"value\": 12.0}}}},\"autotune_result\": {\"c"
  "urrent\": {\"Kp\": 0.415303858749,\"phase_margin\": 80.0,\"wc\": 1481.9804801,\""
  "Ki\": 615.472211978,\"pi_phase_lag\": 45.0},\"velocity\": {\"Kp\": 0.03585026221"
  "28,\"phase_margin\": 65.0,\"wc\": 93.1347734802,\"Ki\": 0.588739222735,\"pi_phas"
  "e_lag\": 10.0}},\"code_generation_tag_map\": {\"application\": \"sample\",\"diag"
  "nostics\": \"x2cscope\"},\"operating_range\": {\"velocity\": {\"units_qualifier"
  "\": \"mechanical\",\"units\": \"rad\\/s\",\"minimum\": 104.7198}},\"build\": {\""
  "hardware_spec\": \"mclv2:33ck256mp508_extopamp_pim\",\"normfactor_associations\""
  ": [{\"context\": null,\"annotations\": [{\"typerefs\": [\"MCAF_U_CURRENT_ABC\","
  "\"MCAF_U_CURRENT_ALPHABETA\",\"MCAF_U_CURRENT_DQ\",\"MCAF_U_CURRENT\",\"MCAF_MOT"
  "OR_DATA.omegaCtrl.outMin\",\"MCAF_MOTOR_DATA.omegaCtrl.outMax\"],\"unit\": \"I\""
  ",\"q\": 15},{\"typerefs\": [\"MCAF_MOTOR_DATA.omegaCtrl.integrator\"],\"unit\": "
  "\"I\",\"q\": 15},{\"typerefs\": [\"MCAF_U_VOLTAGE_ABC\",\"MCAF_U_VOLTAGE_ALPHABE"
  "TA\",\"MCAF_U_VOLTAGE_DQ\",\"MCAF_U_VOLTAGE\",\"MCAF_MOTOR_DATA.idCtrl.outMin\","
  "\"MCAF_MOTOR_DATA.idCtrl.outMax\",\"MCAF_MOTOR_DATA.iqCtrl.outMin\",\"MCAF_MOTOR"
  "_DATA.iqCtrl.outMax\"],\"unit\": \"V\",\"q\": 15},{\"typerefs\": [\"MCAF_MOTOR_D"
  "ATA.idCtrl.integrator\",\"MCAF_MOTOR_DATA.iqCtrl.integrator\"],\"unit\": \"V\","
  "\"q\": 31},{\"typerefs\": [\"MCAF_U_DUTYCYCLE_ABC\"],\"unit\": 1,\"q\": 15},{\"t"
  "yperefs\": [\"MCAF_U_ANGLE_ELEC\"],\"unit\": \"theta_e\",\"q\": 15},{\"typerefs"
  "\": [\"MCAF_U_ANGLE_MECH\"],\"unit\": \"theta_m\",\"q\": 15},{\"typerefs\": [\"M"
  "CAF_U_VELOCITY_MECH\"],\"unit\": \"omega_m\",\"q\": 15},{\"typerefs\": [\"MCAF_U"
  "_VELOCITY_ELEC\"],\"unit\": \"omega_e\",\"q\": 15},{\"typerefs\": [\"MCAF_U_VELO"
  "CITY_DTHETA_ELEC_DT\"],\"unit\": \"dtheta_elec_dt\",\"q\": 15},{\"typerefs\": ["
  "\"MCAF_U_TEMPERATURE\"],\"unit\": \"Temperature\",\"q\": 15},{\"typerefs\": [\"M"
  "CAF_U_DIMENSIONLESS\",\"MCAF_U_DIMENSIONLESS_SINCOS\"],\"unit\": 1,\"q\": 15},{"
  "\"typerefs\": [\"MCAF_U_NORMALIZED_GAIN\"],\"unit\": 1,\"q\": 14}]}]},\"drive\":"
  " {\"iout\": {\"measurement\": {\"compensation\": {\"channels\": [\"a\",\"b\"],\""
  "values\": {\"a\": [1.0,0.0],\"b\": [0.0,1.0]},\"offset\": {\"range\": 0.03125,\""
  "samples\": 128,\"filterGain\": 0.25}}},\"time_constant\": 1.8413e-07,\"fullscale"
  "\": 21.83,\"maximum\": {\"continuous\": 14.1421},\"units\": \"A\"},\"estimator\""
  ": {\"type\": \"pll\",\"runtimeSelect\": false,\"atpll\": {\"tau1\": 0.0001,\"tau"
  "2\": 0.00219,\"kpfactor\": 1.9,\"kifactor\": 30},\"omega1\": 732.0,\"tau\": 0.00"
  "219},\"flux_control\": {\"emag_tau\": 0.001638,\"idref_tau\": 0.006552},\"temper"
  "ature\": {\"fullscale\": 327.68,\"units\": \"C\"},\"pwm\": {\"adc_sampling_delay"
  "\": 0,\"period\": 5e-05,\"duty_cycle\": {\"maximum\": 0.975,\"minimum\": 0.009},"
  "\"deadtime\": 1e-06},\"mcaf\": {\"adc\": {\"triggerSettings\": {\"noTrigger\": "
  "\"NO_TRIGGER\",\"trigger1\": \"PWM1_TRIGGER1\",\"trigger2\": \"PWM1_TRIGGER2\"},"
  "\"channels\": {\"prefix\": \"MCAF_ADC_\",\"critical\": [\"PHASEA_CURRENT\",\"PHA"
  "SEB_CURRENT\",\"PHASEC_CURRENT\",\"DCLINK_CURRENT\",\"DCLINK_VOLTAGE\"],\"requir"
  "ed\": {\"single_channel\": [\"DCLINK_CURRENT\",\"DCLINK_VOLTAGE\",\"POTENTIOMETE"
  "R\"],\"dual_channel\": [\"PHASEA_CURRENT\",\"PHASEB_CURRENT\",\"DCLINK_VOLTAGE\""
  ",\"POTENTIOMETER\"],\"triple_channel\": [\"PHASEA_CURRENT\",\"PHASEB_CURRENT\","
  "\"PHASEC_CURRENT\",\"DCLINK_VOLTAGE\",\"POTENTIOMETER\"]},\"optional\": [\"PHASE"
  "A_VOLTAGE\",\"PHASEB_VOLTAGE\",\"PHASEC_VOLTAGE\",\"BRIDGE_TEMPERATURE\",\"ABSRE"
  "F_VOLTAGE\"],\"fullList\": [{\"name\": \"PHASEA_CURRENT\",\"c_fragment\": \"Ipha"
  "seA\",\"description\": \"phase A current\"},{\"name\": \"PHASEB_CURRENT\",\"c_fr"
  "agment\": \"IphaseB\",\"description\": \"phase B current\"},{\"name\": \"PHASEC_"
  "CURRENT\",\"c_fragment\": \"IphaseC\",\"description\": \"phase C current\"},{\"n"
  "ame\": \"PHASEA_VOLTAGE\",\"c_fragment\": \"VphaseA\",\"description\": \"phase A"
  " voltage\"},{\"name\": \"PHASEB_VOLTAGE\",\"c_fragment\": \"VphaseB\",\"descript"
  "ion\": \"phase B voltage\"},{\"name\": \"PHASEC_VOLTAGE\",\"c_fragment\": \"Vpha"
  "seC\",\"description\": \"phase C voltage\"},{\"name\": \"DCLINK_CURRENT\",\"c_fr"
  "agment\": \"Ibus\",\"description\": \"DC link current\"},{\"name\": \"DCLINK_VOL"
  "TAGE\",\"c_fragment\": \"Dclink\",\"description\": \"DC link voltage\"},{\"name"
  "\": \"POTENTIOMETER\",\"c_fragment\": \"Potentiometer\",\"description\": \"poten"
  "tiometer voltage\"},{\"name\": \"BRIDGE_TEMPERATURE\",\"c_fragment\": \"BridgeTe"
  "mperature\",\"description\": \"bridge temperature voltage\"},{\"name\": \"ABSREF"
  "_VOLTAGE\",\"c_fragment\": \"AbsoluteReferenceVoltage\",\"description\": \"absol"
  "ute reference voltage\"}]}}},\"metadata\": {\"name\": \"dsPIC33CK Low Voltage Mo"
  "tor Control Board\",\"id\": \"lvmc-33ck\"},\"vdc\": {\"nominal\": 24.0,\"time_co"
  "nstant\": 0.0003147,\"fullscale\": 71.3,\"maximum\": 48.0,\"units\": \"V\",\"min"
  "imum\": 12.0},\"configuration\": {\"metadata\": {\"capability\": {\"adc\": {\"ch"
  "annel\": {\"PHASEB_VOLTAGE\": \"AN23\",\"PHASEB_CURRENT\": \"AN1\",\"PHASEA_VOLT"
  "AGE\": \"AN17\",\"PHASEC_CURRENT\": \"AN10\",\"POTENTIOMETER\": \"AN11\",\"BRIDG"
  "E_TEMPERATURE\": \"AN12\",\"DCLINK_VOLTAGE\": \"AN15\",\"PHASEA_CURRENT\": \"AN0"
  "\",\"PHASEC_VOLTAGE\": \"AN22\",\"DCLINK_CURRENT\": \"AN4\"}},\"opamp\": \"inter"
  "nal\",\"buttonCount\": 2.0,\"current_measure\": {\"a\": true,\"b\": true,\"c\": "
  "true,\"dc\": true}},\"scaling\": {\"temperature\": {\"bridge\": {\"offset\": 500"
  "0.0,\"gain\": 0.50354}}},\"peripherals\": {\"adc\": {\"core\": {\"names\": [\"Co"
  "re0\",\"Core1\",\"SharedCore\"]}}},\"pim\": {\"algorithm\": {\"single_channel\":"
  " {\"supported\": true}}},\"device\": \"33ck256mp508\",\"board\": {\"id\": \"lvmc"
  "-33ck\"}},\"peripherals\": {\"adc\": {\"requestedDedCoreSamplingTimeData\": [{\""
  "core\": \"Core0\",\"requestedSamplingTime_us\": 0.3},{\"core\": \"Core1\",\"requ"
  "estedSamplingTime_us\": 0.3}],\"dataOutputFormat\": \"Fractional\",\"channelConf"
  "ig\": [{\"channel\": \"AN0\",\"sign\": \"signed\",\"core\": \"Core0\",\"customNa"
  "me\": \"MCAF_ADC_PHASEA_CURRENT\",\"inverted\": true},{\"channel\": \"AN1\",\"si"
  "gn\": \"signed\",\"core\": \"Core1\",\"customName\": \"MCAF_ADC_PHASEB_CURRENT\""
  ",\"inverted\": true},{\"channel\": \"AN10\",\"sign\": \"signed\",\"core\": \"Sha"
  "red\",\"customName\": \"MCAF_ADC_PHASEC_CURRENT\",\"inverted\": true},{\"channel"
  "\": \"AN4\",\"sign\": \"signed\",\"core\": \"Shared\",\"customName\": \"MCAF_ADC"
  "_DCLINK_CURRENT\",\"inverted\": false},{\"channel\": \"AN15\",\"sign\": \"signed"
  "\",\"core\": \"Shared\",\"customName\": \"MCAF_ADC_DCLINK_VOLTAGE\"},{\"channel"
  "\": \"AN11\",\"sign\": \"signed\",\"core\": \"Shared\",\"customName\": \"MCAF_AD"
  "C_POTENTIOMETER\"},{\"channel\": \"AN12\",\"sign\": \"unsigned\",\"core\": \"Sha"
  "red\",\"customName\": \"MCAF_ADC_BRIDGE_TEMPERATURE\"},{\"channel\": \"AN17\",\""
  "sign\": \"unsigned\",\"core\": \"Shared\",\"customName\": \"MCAF_ADC_PHASEA_VOLT"
  "AGE\"},{\"channel\": \"AN23\",\"sign\": \"unsigned\",\"core\": \"Shared\",\"cust"
  "omName\": \"MCAF_ADC_PHASEB_VOLTAGE\"},{\"channel\": \"AN22\",\"sign\": \"unsign"
  "ed\",\"core\": \"Shared\",\"customName\": \"MCAF_ADC_PHASEC_VOLTAGE\"}],\"custom"
  "Name\": \"MCC_ADC\",\"interruptDriven\": false,\"requestedSharedSamplingTime_us"
  "\": 0.3,\"resolution\": 12.0},\"comparator\": {\"customName\": \"MCC_CMP\",\"dac"
  "OutputEnable\": false,\"nonInvertedSourceSelected\": \"CMP1C\",\"outputPolarity"
  "\": \"Non-Inverted\",\"dacDataValues\": {\"dataHigh\": 3205.0},\"interruptDriven"
  "\": false},\"timer\": [{\"requestedTimerPeriod_ms\": 1.0,\"customName\": \"MCC_T"
  "MR_TICK\",\"interruptDriven\": true},{\"clkSrc\": \"FCY\",\"customName\": \"MCC_"
  "TMR_PROFILE\",\"interruptDriven\": false,\"clkPrescaler\": 1.0,\"periodCount\": "
  "65535.0}],\"oscillator\": {\"clockSource\": \"FRC Oscillator with PLL\",\"reques"
  "tedSystemFrequency_Hz\": 0.0,\"defaultValue\": \"max-frequency\",\"setMaximumSys"
  "temFrequency\": true,\"clockSourceFrequency\": 8000000.0},\"opamp\": [{\"instanc"
  "e\": 1.0,\"enable\": true,\"customName\": \"MCC_OPA_IA\"},{\"instance\": 2.0,\"e"
  "nable\": true,\"customName\": \"MCC_OPA_IB\"},{\"instance\": 3.0,\"enable\": tru"
  "e,\"customName\": \"MCC_OPA_IDC\"}],\"qei\": [{\"noiseFilterEnable\": false,\"pi"
  "nMapping\": [{\"pin\": \"RC12\",\"functionName\": \"A\",\"direction\": \"input\""
  "},{\"pin\": \"RC13\",\"functionName\": \"B\",\"direction\": \"input\"},{\"pin\":"
  " \"RC14\",\"functionName\": \"INDX\",\"direction\": \"input\"}],\"qeiOperationMo"
  "de\": \"Modulo Count mode\",\"indexPulseCaptureEnable\": false,\"homePulseCaptur"
  "eEnable\": false,\"homePulsePolarity\": false,\"customName\": \"MCC_QEI\",\"inde"
  "xPulsePolarity\": false}],\"wdt\": {\"wdtEnable\": true,\"requestedWdtPeriod_ms"
  "\": 1.0,\"wdtMode\": \"Non-Window mode\",\"wdtEnableType\": \"Software\"},\"pwm"
  "\": {\"synchronousUpdate\": true,\"duty_cycle\": {\"maximum\": 0.975,\"minimum\""
  ": 0.009},\"requestedDeadTime_us\": {\"defaultValue\": 1.0,\"maximum\": 6.0,\"min"
  "imum\": 0.67},\"customName\": \"MCC_PWM\",\"generatorMapping\": [{\"name\": \"MO"
  "TOR1_PHASE_A\",\"generator\": 1.0},{\"name\": \"MOTOR1_PHASE_B\",\"generator\": "
  "2.0},{\"name\": \"MOTOR1_PHASE_C\",\"generator\": 4.0}],\"interruptDriven\": fal"
  "se,\"faultInputPolarity\": \"Active-high\",\"faultInput\": {},\"frequency\": {\""
  "defaultValue\": 20000.0,\"maximum\": 100000.0,\"minimum\": 1000.0},\"faultInputS"
  "ource\": \"CMP1 Output\",\"polarity\": {\"lower\": \"Active-high\",\"upper\": \""
  "Active-high\"}},\"uart\": {\"baudRate\": 115200.0,\"pinMapping\": [{\"name\": \""
  "TX\",\"pinSelected\": \"RD13\"},{\"name\": \"RX\",\"pinSelected\": \"RD14\"}],\""
  "stopBits\": 1.0,\"parity\": \"None\",\"dataSize\": 8.0,\"customName\": \"MCC_UAR"
  "T\",\"interruptDriven\": false},\"gpio\": [{\"isWeakPullUp\": false,\"pinSelecte"
  "d\": \"RE6\",\"interruptOnChange\": \"none\",\"isWeakPullDown\": false,\"customN"
  "ame\": \"MCAF_LED1\",\"isAnalogPin\": false,\"direction\": \"output\",\"isOpenDr"
  "ain\": false},{\"isWeakPullUp\": false,\"pinSelected\": \"RE7\",\"interruptOnCha"
  "nge\": \"none\",\"isWeakPullDown\": false,\"customName\": \"MCAF_LED2\",\"isAnal"
  "ogPin\": false,\"direction\": \"output\",\"isOpenDrain\": false},{\"isWeakPullUp"
  "\": false,\"pinSelected\": \"RE11\",\"interruptOnChange\": \"none\",\"isWeakPull"
  "Down\": false,\"customName\": \"MCAF_BUTTON1\",\"isAnalogPin\": false,\"directio"
  "n\": \"input\",\"isOpenDrain\": false},{\"isWeakPullUp\": false,\"pinSelected\":"
  " \"RE12\",\"interruptOnChange\": \"none\",\"isWeakPullDown\": false,\"customName"
  "\": \"MCAF_BUTTON2\",\"isAnalogPin\": false,\"direction\": \"input\",\"isOpenDra"
  "in\": false},{\"isWeakPullUp\": false,\"pinSelected\": \"RE4\",\"interruptOnChan"
  "ge\": \"none\",\"isWeakPullDown\": false,\"customName\": \"MCAF_TESTPOINT1\",\"i"
  "sAnalogPin\": false,\"direction\": \"output\",\"isOpenDrain\": false}]},\"displa"
  "yName\": \"dsPIC33CK LVMC\",\"name\": \"dsPIC33CK Low Voltage Motor Control (LVM"
  "C) Development Board\",\"partNumber\": \"DM330031\",\"id\": \"lvmc-33ck\",\"supp"
  "ortedPim\": []},\"idc\": {\"maximum\": 2.9,\"units\": \"A\"},\"sampling_time\": "
  "{\"current\": 5e-05,\"velocity\": 0.001},\"processor\": {\"clock_frequency\": 10"
  "0000000.0,\"pim\": \"PIM\\/DIM Not Applicable\",\"familyName\": \"dsPIC33CK256MP"
  "508\",\"device\": \"dsPIC33CK256MP508\"},\"vbus\": 24.0},\"config\": {\"ui\": {"
  "\"content\": {\"current_measure-method\": \"dual_channel\",\"current_measure-sin"
  "gle_channel-minimumTime\": 2.0,\"current_measure-single_channel-sampleDelay\": 0"
  ".0,\"current_measure-opamp-full_input_range\": true,\"estimator-type\": \"pll\","
  "\"estimator-active-pll\": false,\"estimator-active-qei\": false,\"estimator-acti"
  "ve-atpll\": false,\"estimator-active-zsmt\": false,\"estimator-active-ideal\": f"
  "alse,\"operation-min_velocity_ratio\": 0.25,\"operation-startup_velocity_ratio\""
  ": 0.2744,\"operation-max_velocity_ratio\": 1.25,\"operation-fullscale_base_ratio"
  "\": 1.5,\"operation-outer_loop_type\": \"velocity\",\"operation-saliency-thresho"
  "ld\": 1.25,\"operation-slewrate-accel\": 0.5,\"operation-slewrate-decel\": 1.0,"
  "\"operation-dyn_current_type\": \"none\",\"operation-stopping-type\": \"minimal_"
  "impact\",\"operation-stopping-closed_loop_parameters-speed\": 0.05,\"operation-s"
  "topping-closed_loop_parameters-time\": 0.5,\"operation-coastdown-end_velocity\":"
  " 0.05,\"operation-coastdown-time\": 1.2,\"fault_inject-get_tf-div0\": false,\"fa"
  "ult_inject-get_tf-missing_data\": false,\"fault_inject-get_tf-raise_value_error"
  "\": false,\"fault_inject-get_tf-raise_custom_error\": false,\"fault_inject-tf-tf"
  "_null\": false,\"fault_inject-tf-tf_jy\": false,\"fault_inject-tf-tf_jyex\": fal"
  "se,\"fault_inject-tf-div0\": false,\"fault_inject-tf-tau\": 0.0,\"flux_control-m"
  "ethod\": \"none\",\"flux_control-eqn_based-ilimit-region_type\": \"quadratic\","
  "\"flux_control-eqn_based-ilimit-id_limit\": 0.7,\"flux_control-eqn_based-ilimit-"
  "iq_limit\": 0.95,\"flux_control-eqn_based-fluxweak_enable\": false,\"flux_contro"
  "l-eqn_based-fw-vdq_limit\": 0.95,\"flux_control-eqn_based-mtpa_enable\": false,"
  "\"deadtimecomp-method\": \"none\",\"deadtimecomp-perphase-current_sign_band\": 0"
  ".02,\"deadtimecomp-perphase-forward_gain\": 0.0,\"deadtimecomp-perphase-feedback"
  "_gain\": 0.0,\"fault_detect-margin_uv\": 2.0,\"fault_detect-margin_ov\": 2.0,\"f"
  "ault_detect-stall_detect_enable\": true,\"startup-Istartup\": 0.75,\"startup-t_r"
  "ampup\": 25.0,\"startup-t_align\": 0.0,\"startup-min_total_accel_time\": 250.0,"
  "\"startup-torque_scale_accel\": 0.15,\"startup-torque_scale_slow_accel\": 0.2,\""
  "startup-t_hold\": 0.0,\"startup-omega0\": 0.2,\"startup-theta_converge_rate\": 1"
  ".0,\"startup-method\": \"classic\",\"startup-damping-Imax\": 0.1,\"startup-dampi"
  "ng-gainmax\": 40.0,\"startup-damping-omega_min\": 0.4,\"overmodulation-vd_limit"
  "\": 1.0,\"overmodulation-vq_limit\": 1.15,\"overmodulation-include_duty_clipping"
  "\": true,\"pll-tau\": 2.19,\"pll-omega1\": 116.5,\"pll-velocity_filter_threshold"
  "\": 1.0,\"encoder-lines\": 250.0,\"encoder-index_present\": false,\"encoder-trac"
  "king_loop-tau\": 1.5,\"encoder-qei_sync-method\": \"align\",\"encoder-qei_sync-a"
  "lign-t_align\": 0.5,\"encoder-qei_sync-align-angle_shift\": 30.0,\"encoder-qei_s"
  "ync-align-angle_init\": -30.0,\"encoder-qei_sync-align_sweep-sweep_rate_pow\": 1"
  ".0,\"encoder-qei_sync-align_sweep-setup_angle\": 45.0,\"encoder-qei_sync-pullout"
  "-pullout_slip\": 0.117,\"zsmt-excitation-kV\": 0.12,\"zsmt-excitation-kI\": 0.1,"
  "\"zsmt-pll-zeta\": 1.5,\"zsmt-pll-tau\": 10.0,\"zsmt-pll-tau_lpf\": 0.0,\"zsmt-p"
  "ll-execution_divider\": 2.0,\"zsmt-startup-align_delay\": 1.0,\"zsmt-startup-pll"
  "_lock_time\": 1.0,\"zsmt-startup-speed_limit\": 0.015,\"zsmt-startup-current_lim"
  "it\": 0.3,\"zsmt-startup-probe_duration\": 10.0,\"zsmt-startup-probe_slew_time\""
  ": 1.5,\"zsmt-startup-probe_blanking_time\": 4.0,\"zsmt-hybrid-type\": \"minotaur"
  "\",\"zsmt-minotaur-voltage_limit-vd\": 0.45,\"zsmt-minotaur-voltage_limit-vq\": "
  "0.78,\"zsmt-minotaur-converge-angle\": 10.0,\"zsmt-minotaur-converge-duration\":"
  " 2.0,\"zsmt-minotaur-velocity_threshold-slow\": 0.15,\"zsmt-minotaur-velocity_th"
  "reshold-transition\": 0.2,\"zsmt-minotaur-velocity_threshold-fast\": 0.25,\"zsmt"
  "-angle_correction-current_gain\": 0.0,\"dyn_current1-peak_factor\": 1.0,\"dyn_cu"
  "rrent1-horizon_factor\": 1.05,\"dyn_current1-tau\": 5.0,\"dyn_current1-nsamples"
  "\": 128.0,\"voltage_outerloop-tau_lpf\": 2.0,\"voltage_outerloop-Kp\": 0.1,\"vol"
  "tage_outerloop-tau\": 100.0,\"mcapi-isSquaredTau\": 1.0,\"mcapi-iqTau\": 1.0,\"m"
  "capi-adcIsrUserFunctions_enable\": false,\"board_service-uiServiceTiming\": 1.0,"
  "\"board_service-uiButtonDebounceTime\": 7.0,\"board_service-uiButtonLongPressTim"
  "e\": 2.5,\"test_harness-perturbation_asymmetric\": false,\"adc-vtempbridge\": fa"
  "lse,\"adc-temp_bridge_params-threshold\": 50.0,\"adc-temp_bridge_params-tau_lpf"
  "\": 10.0,\"adc-temp_bridge_params-slewrate\": 4.0,\"adc-vabsref\": false,\"adc-v"
  "phasea\": false,\"adc-vphaseb\": false,\"adc-vphasec\": false},\"version\": {\"s"
  "chema\": 4.0,\"motorBench\": \"2.45.0\",\"MCAF\": \"R7\"},\"status\": {\"valid\""
  ": true}}}}";