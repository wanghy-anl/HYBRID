within NHES.ExperimentalSystems.MAGNET_TEDS;
model TEDS_MAGNET_Integration_GT_0_1_bypass
  "Dynamic simulation of the integrated system of MAGNET, TEDS, and Gas turbine with a central control system"
  extends TRANSFORM.Icons.Example;

protected
  inner TRANSFORM.Fluid.SystemTF systemTF(
    showColors=true,
    val_min=data.T_hx_co,
    val_max=data.T_vc_rp)
    annotation (Placement(transformation(extent={{-360,-160},{-340,-140}})));

protected
  inner TRANSFORM.Fluid.System system(
    p_ambient=18000,
    T_ambient=498.15,
    m_flow_start=0.84)
    annotation (Placement(transformation(extent={{-360,-200},{-340,-180}})));
protected
  NHES.ExperimentalSystems.MAGNET.Data.Data_base_An data(eta_mech=0.4)
    annotation (Placement(transformation(extent={{-360,-120},{-340,-100}})));
protected
  package Medium = Modelica.Media.IdealGases.SingleGases.N2;//TRANSFORM.Media.ExternalMedia.CoolProp.Nitrogen;
  package Medium_cw = Modelica.Media.Water.StandardWater;
  package Medium_TEDS =
      TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C;

protected
  TRANSFORM.Fluid.BoundaryConditions.Boundary_pT boundary(
    redeclare package Medium = Medium_cw,
    p=data.p_hx_cw,
    T=data.T_hx_cw,
    nPorts=1)
    annotation (Placement(transformation(extent={{32,-90},{12,-70}})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort sensor_hx_cw(
    redeclare package Medium = Medium_cw,
    p_start=data.p_hx_cw,
    T_start=data.T_hx_cw,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-48,-90},{-28,-70}})));
protected
  TRANSFORM.HeatExchangers.Simple_HX hx(
    redeclare package Medium_1 = Medium,
    redeclare package Medium_2 = Medium_cw,
    nV=10,
    V_1=1,
    V_2=1,
    UA=data.UA_hx,
    p_a_start_1=data.p_rp_hx,
    p_b_start_1=data.p_hx_co,
    T_a_start_1=data.T_rp_hx,
    T_b_start_1=data.T_hx_co,
    m_flow_start_1=data.m_flow,
    p_a_start_2=data.p_cw_hx,
    p_b_start_2=data.p_hx_cw,
    T_a_start_2=data.T_cw_hx,
    T_b_start_2=data.T_hx_cw,
    m_flow_start_2=data.m_flow_cw,
    R_1=-data.dp_hx_hot/data.m_flow)
    annotation (Placement(transformation(extent={{-78,-92},{-58,-112}})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort sensor_cw_hx(
    redeclare package Medium = Medium_cw,
    p_start=data.p_cw_hx,
    T_start=data.T_cw_hx,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-112,-88},{-92,-68}})));
protected
  TRANSFORM.Fluid.BoundaryConditions.MassFlowSource_T m_source_cw(
    redeclare package Medium = Medium_cw,
    use_m_flow_in=true,
    m_flow=data.m_flow_cw,
    T=data.T_cw_hx,
    nPorts=1)
    annotation (Placement(transformation(extent={{-194,-88},{-174,-68}})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort sensor_rp_hx_2(
    redeclare package Medium = Medium,
    p_start=data.p_rp_hx,
    T_start=data.T_rp_hx,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-24,-128},{-44,-108}})));
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort sensor_hx_co(
    redeclare package Medium = Medium,
    p_start=data.p_hx_co,
    T_start=data.T_hx_co,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-90,-128},{-110,-108}})));
protected
  TRANSFORM.Fluid.Valves.ValveIncompressible valve_ps(
    redeclare package Medium = Medium,
    dp_nominal(displayUnit="Pa") = 1e4,
    m_flow_nominal=1,
    opening_nominal=0.5)
    annotation (Placement(transformation(extent={{-192,-128},{-172,-108}})));
  Modelica.Blocks.Sources.Constant opening_valve_tank(k=1)
    annotation (Placement(transformation(extent={{-226,-108},{-206,-88}})));
  TRANSFORM.HeatExchangers.Simple_HX  rp(
    redeclare package Medium_1 = Medium,
    redeclare package Medium_2 = Medium,
    nV=10,
    V_1=1,
    V_2=1,
    UA=data.UA_rp,
    p_a_start_1=data.p_vc_rp,
    p_b_start_1=data.p_rp_hx,
    T_a_start_1=data.T_vc_rp,
    T_b_start_1=data.T_rp_hx,
    m_flow_start_1=data.m_flow,
    p_a_start_2=data.p_co_rp,
    p_b_start_2=data.p_rp_vc,
    T_a_start_2=data.T_co_rp,
    T_b_start_2=data.T_rp_vc,
    m_flow_start_2=data.m_flow,
    R_1=-data.dp_rp_hot/data.m_flow,
    R_2=-data.dp_rp_cold/data.m_flow)
    annotation (Placement(transformation(extent={{-76,-32},{-56,-12}})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_co_rp_1(
    redeclare package Medium = Medium,
    p_start=data.p_co_rp,
    T_start=data.T_co_rp,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    precision2=1,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-16,-50},{-36,-30}})));
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_rp_hx_1(
    redeclare package Medium = Medium,
    p_start=data.p_rp_hx,
    T_start=data.T_rp_hx,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-36,-12},{-16,8}})));
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_vc_pipe_rp(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-108,-12},{-88,8}})));
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_rp_pipe_vc(
    redeclare package Medium = Medium,
    p_start=data.p_rp_vc,
    T_start=data.T_rp_vc,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-86,-50},{-106,-30}})));
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_vc_pipe(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    precision2=3,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-268,-12},{-248,8}})));
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_pipe_vc(
    redeclare package Medium = Medium,
    p_start=data.p_rp_vc,
    T_start=data.T_rp_vc,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(extent={{-188,-50},{-208,-30}})));
protected
  TRANSFORM.Fluid.Volumes.SimpleVolume vc(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
        (V=0.1),
    Q_gen=data.Q_vc)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-286,-32})));
  TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance resistance(
      redeclare package Medium = Medium, R=-data.dp_vc/data.m_flow)
                                         annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-286,-14})));
  TRANSFORM.Fluid.Volumes.SimpleVolume volume_co(
    redeclare package Medium = Medium,
    p_start=data.p_hx_co,
    T_start=data.T_hx_co,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
        (V=0.01),
    Q_gen=0) "12022.6"
                  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-88,-182})));
public
  TRANSFORM.Fluid.Sensors.MassFlowRate mflow_MAGNET(
    redeclare package Medium = Medium,
    p_start=data.p_co_rp,
    T_start=data.T_co_rp,
    precision=2)
    annotation (Placement(transformation(extent={{-24,-192},{-4,-172}})));
protected
  TRANSFORM.Fluid.BoundaryConditions.Boundary_pT ps(
    redeclare package Medium = Medium,
    p=data.p_hx_co,
    T=data.T_ps,
    nPorts=1)
    annotation (Placement(transformation(extent={{-262,-128},{-242,-108}})));
  TRANSFORM.Fluid.Pipes.GenericPipe_MultiTransferSurface pipe_hx_co(
    redeclare package Medium = Medium,
    p_a_start=data.p_hx_co,
    T_a_start=data.T_hx_co,
    m_flow_a_start=data.m_flow,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
        (dimension=data.d_hx_co, length=data.length_hx_co))
    annotation (Placement(transformation(extent={{-142,-192},{-122,-172}})));
  TRANSFORM.Fluid.Pipes.GenericPipe_MultiTransferSurface pipe_co_rp(
    redeclare package Medium = Medium,
    p_a_start=data.p_co_rp,
    T_a_start=data.T_co_rp,
    m_flow_a_start=data.m_flow,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
        (dimension=data.d_co_rp, length=data.length_co_rp))
    annotation (Placement(transformation(extent={{12,-50},{-8,-30}})));
  TRANSFORM.Fluid.Machines.Pump_Controlled co(
    redeclare package Medium = Medium,
    p_a_start=data.p_hx_co,
    p_b_start=data.p_co_rp,
    T_a_start=data.T_hx_co,
    T_b_start=data.T_co_rp,
    m_flow_start=data.m_flow,
    redeclare model EfficiencyChar =
        TRANSFORM.Fluid.Machines.BaseClasses.PumpCharacteristics.Efficiency.Constant
        (eta_constant=0.7027),
    controlType="m_flow",
    use_port=true)
    annotation (Placement(transformation(extent={{-64,-192},{-44,-172}})));
  TRANSFORM.Fluid.Pipes.GenericPipe_withWallAndInsulation pipe_vc_TEDS(
    ths_wall=fill(data.th_4in_sch40, pipe_vc_TEDS.geometry.nV),
    ths_insulation=fill(data.th_4in_sch40, pipe_vc_TEDS.geometry.nV),
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
        (dimension=data.d_vc_rp, length=data.length_vc_rp/2),
    redeclare package Medium = Medium,
    p_a_start=data.p_vc_rp,
    T_a_start=data.T_vc_rp,
    m_flow_a_start=data.m_flow,
    redeclare package Material_wall = TRANSFORM.Media.Solids.SS316)
    annotation (Placement(transformation(extent={{-240,-12},{-220,8}})));
  TRANSFORM.Fluid.Pipes.GenericPipe_withWallAndInsulation pipe_ins_rp_vc(
    ths_wall=fill(data.th_4in_sch40, pipe_vc_TEDS.geometry.nV),
    ths_insulation=fill(data.th_4in_sch40, pipe_vc_TEDS.geometry.nV),
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
        (dimension=data.d_rp_vc, length=data.length_rp_vc),
    redeclare package Medium = Medium,
    p_a_start=data.p_rp_vc,
    T_a_start=data.T_rp_vc,
    m_flow_a_start=data.m_flow)
    annotation (Placement(transformation(extent={{-152,-50},{-172,-30}})));
  TRANSFORM.Fluid.Pipes.GenericPipe_withWallAndInsulation pipe_ins_rp_hx(
    ths_wall=fill(data.th_4in_sch40, pipe_vc_TEDS.geometry.nV),
    ths_insulation=fill(data.th_4in_sch40, pipe_vc_TEDS.geometry.nV),
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
        (dimension=data.d_rp_hx, length=data.length_rp_hx),
    redeclare package Medium = Medium,
    p_a_start=data.p_rp_hx,
    T_a_start=data.T_rp_hx,
    m_flow_a_start=data.m_flow)
    annotation (Placement(transformation(extent={{22,-128},{2,-108}})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort T_vc_TEDS(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-204,56})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_TEDS_rp(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-140,48})));
public
  TRANSFORM.Fluid.Sensors.MassFlowRate m_flow_vc_TEDS(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    precision=2)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-204,84})));
protected
  TRANSFORM.Fluid.Pipes.GenericPipe_withWallAndInsulation pipe_vc_rp(
    ths_wall=fill(data.th_4in_sch40, pipe_vc_TEDS.geometry.nV),
    ths_insulation=fill(data.th_4in_sch40, pipe_vc_TEDS.geometry.nV),
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.StraightPipe
        (dimension=data.d_vc_rp, length=data.length_vc_rp/2),
    redeclare package Medium = Medium,
    p_a_start=data.p_vc_rp,
    T_a_start=data.T_vc_rp,
    m_flow_a_start=data.m_flow,
    redeclare package Material_wall = TRANSFORM.Media.Solids.SS316,
    pipe(flowModel(dps_fg(start={-128.03918155874314}))))
    annotation (Placement(transformation(extent={{-138,-12},{-118,8}})));
  TRANSFORM.Fluid.Volumes.SimpleVolume volume_MT(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
        (V=0.01),
    Q_gen=0) "12022.6" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-140,18})));
protected
  Modelica.Blocks.Sources.RealExpression Tout_vc(y=pT_vc_pipe.T)
    annotation (Placement(transformation(extent={{64,156},{82,174}})));
  Modelica.Blocks.Sources.RealExpression Tin_vc(y=pT_pipe_vc.T)
    annotation (Placement(transformation(extent={{64,144},{82,162}})));
public
  TRANSFORM.Fluid.Valves.ValveLinear valve_vc_TEDS(
    redeclare package Medium = Medium,
    dp_nominal=3000,
    m_flow_nominal=1) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-204,18})));
  TRANSFORM.Fluid.Valves.ValveLinear valve_TEDS_rp(
    redeclare package Medium = Medium,
    dp_nominal=3000,
    m_flow_nominal=1) annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=90,
        origin={-140,70})));
  TRANSFORM.Fluid.Sensors.MassFlowRate mflow_cw(
    redeclare package Medium = Medium_cw,
    p_start=data.p_co_rp,
    T_start=data.T_co_rp,
    precision=3)
    annotation (Placement(transformation(extent={{-150,-88},{-130,-68}})));
  Modelica.Blocks.Sources.RealExpression mflow_inside_MAGNET(y=mflow_MAGNET.m_flow)
    annotation (Placement(transformation(extent={{64,168},{82,184}})));
  GasTurbine.Turbine.Turbine turbine(
    redeclare package Medium = Medium,
    explicitIsentropicEnthalpy=true,
    Tstart_in=data.T_vc_rp,
    Tstart_out=489.15,
    PR0=2.975,
    w0=data.m_flow/2)
              annotation (Placement(transformation(
        extent={{-22,-17},{22,17}},
        rotation=90,
        origin={-327,112})));
  TRANSFORM.Fluid.Valves.ValveLinear valve_vc_GT(
    redeclare package Medium = Medium,
    dp_nominal=3000,
    m_flow_nominal=2) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=90,
        origin={-340,36})));
  TRANSFORM.Fluid.Sensors.MassFlowRate mflow_vc_GT(
    redeclare package Medium = Medium,
    p_start=data.p_co_rp,
    T_start=data.T_co_rp,
    precision=3)
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=90,
        origin={-340,84})));
  Modelica.Blocks.Sources.RealExpression GT_Power(y=data.eta_mech*(turbine.Wt
         - compressor.Wc))
    annotation (Placement(transformation(extent={{64,178},{82,194}})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_GT_co(
    redeclare package Medium = Medium,
    p_start=data.p_atm,
    T_start=data.T_vc_rp,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-340,148})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_co_rp(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-240,172})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_vc_GT(
    redeclare package Medium = Medium,
    p_start=data.p_atm,
    T_start=data.T_vc_rp,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-340,58})));
  TRANSFORM.HeatAndMassTransfer.BoundaryConditions.Heat.Temperature    boundary3(use_port=
        false, T=306.15)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-364,186})));
  GasTurbine.Compressor.Compressor compressor(
    redeclare package Medium = Medium,
    pstart_in=data.p_atm,
    Tstart_in=306.15,
    Tstart_out=723.15,
    PR0=2.975,
    w0=data.m_flow/2)
    annotation (Placement(transformation(extent={{-286,150},{-250,174}})));
public
  TRANSFORM.Fluid.Sensors.PressureTemperatureTwoPort pT_co_rp1(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp,
    T_start=data.T_vc_rp,
    precision=1,
    redeclare function iconUnit =
        TRANSFORM.Units.Conversions.Functions.Pressure_Pa.to_bar,
    redeclare function iconUnit2 =
        TRANSFORM.Units.Conversions.Functions.Temperature_K.to_degC)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-300,172})));
  Systems.BalanceOfPlant.StagebyStageTurbineSecondary.Control_and_Distribution.SpringBallValve
    springBallValve(
    redeclare package Medium = Medium,
    p_spring=data.P_Release,
    K=1,
    opening_init=0.)
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=90,
        origin={-318,188})));
  TRANSFORM.Fluid.BoundaryConditions.Boundary_ph boundary5(
    redeclare package Medium = Medium,
    p=data.P_Release,
    nPorts=1)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-268,200})));
  TRANSFORM.Fluid.Sensors.MassFlowRate mflow_GT_rp(
    redeclare package Medium = Medium,
    p_start=data.p_co_rp,
    T_start=data.T_co_rp,
    precision=3) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={-214,154})));
  TRANSFORM.Fluid.Volumes.SimpleVolume Cooler(
    redeclare package Medium = Medium,
    p_start=data.p_vc_rp/2.975,
    T_start=306.15,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
        (V=0),
    use_HeatPort=true) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-332,172})));
public
  TRANSFORM.HeatExchangers.Simple_HX MAGNET_TEDS_simpleHX1(
    redeclare package Medium_1 = Medium,
    redeclare package Medium_2 =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    nV=10,
    V_1=1,
    V_2=1,
    UA=data.UA_MAGNET_TEDS,
    p_a_start_1=data.p_vc_rp + 100,
    p_b_start_1=data.p_vc_rp,
    T_a_start_1=data.T_vc_rp,
    T_b_start_1=502.15,
    m_flow_start_1=data.m_flow,
    p_a_start_2=data.p_TEDS_in,
    p_b_start_2=data.p_TEDS_out,
    T_a_start_2=data.T_cold_side,
    T_b_start_2=data.T_hot_side,
    m_flow_start_2=data.m_flow_TEDS)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={74,102})));
  Modelica.Fluid.Pipes.DynamicPipe pipe2(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=1,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.1),
    T_start=data.T_hot_side)
    annotation (Placement(transformation(extent={{212,100},{228,116}})));
  Modelica.Fluid.Pipes.DynamicPipe pipe4(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=1,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.1),
    T_start=data.T_cold_side)
                    annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=90,
        origin={78,-42})));
  Modelica.Fluid.Pipes.DynamicPipe pipe7(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    length=0.1,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=600, m_flow_nominal=0.1))
                    annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=270,
        origin={360,14})));
  TRANSFORM.Fluid.Volumes.ExpansionTank tank1(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    A=1,
    V0=0.1,
    p_surface=system.p_ambient,
    p_start=system.p_start,
    level_start=0,
    h_start=Medium.specificEnthalpy_pT(system.p_start, data.T_hot_side))
    annotation (Placement(transformation(extent={{180,104},{200,124}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_002(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    T_start=data.T_cold_side,
    precision=3) annotation (Placement(transformation(
        extent={{-12,-11},{12,11}},
        rotation=90,
        origin={78,-7})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_003(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    T_start=data.T_hot_side,
    precision=3)
    annotation (Placement(transformation(extent={{124,96},{150,120}})));
  TRANSFORM.Fluid.Sensors.MassFlowRate m_Chg_In(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={238,76})));
  TRANSFORM.Fluid.Sensors.MassFlowRate sensor_m_flow2(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision=
       3) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={360,80})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_049(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    m_flow_start=1e-2,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={238,94})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_050(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={308,94})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_051(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=-90,
        origin={238,-100})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_052(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={308,-80})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_006(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=false,
    m_flow_start=0.41,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={286,108})));
  TRANSFORM.Fluid.Sensors.MassFlowRate FM_201(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{9,9},{-9,-9}},
        rotation=-90,
        origin={239,-81})));
  TRANSFORM.Fluid.Sensors.MassFlowRate FM_202(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-9,-10},{9,10}},
        rotation=-90,
        origin={308,-95})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_004(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    m_flow_start=0.41,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={102,-130})));
  TRANSFORM.Fluid.Sensors.MassFlowRate FM_003(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    T_start=data.T_cold_side,
    precision=3)
    annotation (Placement(transformation(extent={{132,-138},{114,-122}})));
  TRANSFORM.Fluid.Sensors.MassFlowRate m_Dch_Out(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={308,76})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_Chg_Out(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-9,10},{9,-10}},
        rotation=90,
        origin={308,-59})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_Chg_In(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={238,50})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_Dch_Out(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={308,50})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_Dch_In(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-9.5,-8.5},{9.5,8.5}},
        rotation=90,
        origin={239,-59.5})));
  TRANSFORM.Fluid.Sensors.MassFlowRate BOP_Mass_flow(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    T_start=data.T_hot_side,
    precision=3)
          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={260,108})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_chiller_before(redeclare package
      Medium = TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
      precision=3) annotation (Placement(transformation(
        extent={{-13,12},{13,-12}},
        rotation=270,
        origin={360,47})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort T_chiller_after(redeclare package
      Medium = TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
      precision=3) annotation (Placement(transformation(
        extent={{-12,12},{12,-12}},
        rotation=270,
        origin={360,-32})));
  Systems.Experiments.TEDS.ThermoclineModels.Thermocline_Insulation_An
    thermocline_Insulation_An(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    redeclare package InsulationMaterial = Media.Solids.FoamGlass,
    geometry(
      Radius_Tank=0.438,
      Porosity=0.5,
      nodes=30,
      dr=0.00317,
      Insulation_thickness=3*0.051,
      Wall_Thickness=0.019,
      Height_Tank=4.435))
    annotation (Placement(transformation(extent={{260,-20},{288,12}})));
  Systems.Experiments.TEDS.BaseClasses.SignalSubBus_ActuatorInput sensorSubBus
    annotation (Placement(transformation(extent={{152,126},{174,150}})));
  Systems.Experiments.TEDS.BaseClasses.SignalSubBus_SensorOutput actuatorSubBus
    annotation (Placement(transformation(extent={{184,126},{206,150}})));
  Modelica.Blocks.Sources.RealExpression MAGNET_heater_input(y=
        MAGNET_TEDS_simpleHX1.Q_flow)
    annotation (Placement(transformation(extent={{64,188},{82,204}})));
  TRANSFORM.Fluid.Machines.Pump_PressureBooster P_001(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    use_input=true,
    p_nominal=system.p_ambient + 1e4)
    annotation (Placement(transformation(extent={{164,-138},{148,-122}})));
  Magnet_TEDS.MAGNET_TEDS_ControlSystem.Control_System_Therminol_4_element_all_modes_MAGNET_GT_dyn_0_1_bypass
    control_System_Therminol_4_element_all_modes_MAGNET_GT_dyn_0_1_HW(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    TEDS_Heat_Load_Demand=TEDS_Heat_Demand_W.y,
    Gas_Turbine_Elec_Demand=PCU_Elec_Demand_W.y,
    T_hot_design=598.15,
    T_cold_design=498.15)
    annotation (Placement(transformation(extent={{222,162},{256,196}})));
  Modelica.Blocks.Sources.Pulse          TEDS_Heat_Demand_kW(
    amplitude=15,
    period=7200,
    offset=35.59071,
    startTime=10800)
    annotation (Placement(transformation(extent={{112,2},{126,16}})));
  Modelica.Blocks.Math.Gain TEDS_Heat_Demand_W(k=1000)
    annotation (Placement(transformation(extent={{136,4},{146,14}})));
  Modelica.Blocks.Sources.Pulse    PCU_Elec_Demand_kW(
    amplitude=5,
    period=7200,
    offset=10,
    startTime=5400)
    annotation (Placement(transformation(extent={{112,-26},{126,-12}})));
  Modelica.Blocks.Math.Gain PCU_Elec_Demand_W(k=1000)
    annotation (Placement(transformation(extent={{136,-24},{146,-14}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_201(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-9,10},{9,-10}},
        rotation=-90,
        origin={274,25})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_202(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-9,-10},{9,10}},
        rotation=90,
        origin={274,-31})));
  Modelica.Fluid.Pipes.DynamicPipe pipe3(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=1,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=700, m_flow_nominal=0.84))
                    annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=0,
        origin={352,-130})));
  TRANSFORM.Fluid.Sensors.MassFlowRate FM_002(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{10.5,11},{-10.5,-11}},
        rotation=0,
        origin={305.5,-169})));
  Modelica.Fluid.Sources.MassFlowSource_T Chiller_Mass_Flow(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.EthyleneGlycol.LinearEthyleneGlycol_50_Water,
    use_m_flow_in=true,
    m_flow=12.6,
    T=280.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{186,-208},{206,-188}})));
  Modelica.Fluid.Sources.Boundary_pT boundary1(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.EthyleneGlycol.LinearEthyleneGlycol_50_Water,
    p=300000,
    T=291.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{340,-180},{320,-200}})));
  TRANSFORM.HeatExchangers.GenericDistributed_HX HX_001(
    p_b_start_shell=system.p_ambient,
    T_a_start_shell=data.T_hot_side,
    T_b_start_shell=data.T_cold_side,
    p_b_start_tube=boundary1.p,
    counterCurrent=true,
    m_flow_a_start_tube=Chiller_Mass_Flow.m_flow,
    m_flow_a_start_shell=12.6,
    redeclare package Medium_tube =
        TRANSFORM.Media.Fluids.EthyleneGlycol.LinearEthyleneGlycol_50_Water,
    redeclare package Medium_shell =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    redeclare package Material_tubeWall = TRANSFORM.Media.Solids.SS316,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.DistributedVolume_1D.HeatExchanger.ShellAndTubeHX
        (
        D_o_shell=0.192,
        nV=10,
        nTubes=113,
        nR=3,
        length_shell=3.0,
        dimension_tube=0.013,
        length_tube=3.0,
        th_wall=0.0001),
    p_a_start_tube=boundary1.p + 100,
    T_a_start_tube=Chiller_Mass_Flow.T,
    T_b_start_tube=m_source_cw.T,
    p_a_start_shell=system.p_ambient + 100,
    redeclare model HeatTransfer_tube =
        TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Nus_DittusBoelter_Simple
        (CF=1.0),
    redeclare model HeatTransfer_shell =
        TRANSFORM.Fluid.ClosureRelations.HeatTransfer.Models.DistributedPipe_1D_MultiTransferSurface.Nus_DittusBoelter_Simple
        (CF=2.0))
    annotation (Placement(transformation(extent={{231,-192},{262,-162}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort Ethylene_glycol_exit_temperature(
      redeclare package Medium =
        TRANSFORM.Media.Fluids.EthyleneGlycol.LinearEthyleneGlycol_50_Water,
      precision=3)
    annotation (Placement(transformation(extent={{280,-202},{304,-178}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_006(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(extent={{178,-142},{202,-118}})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_004(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-9,12},{9,-12}},
        rotation=0,
        origin={280,-169})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_012(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=180,
        origin={174,-170})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_009(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=3000,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{6,-6},{-6,6}},
        rotation=0,
        origin={256,-130})));
  Fluid.Pipes.NonLinear_Break nonLinear_Break(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C)
    annotation (Placement(transformation(extent={{6,-8},{-6,8}},
        rotation=0,
        origin={222,-170})));
  Fluid.Pipes.NonLinear_Break nonLinear_Break2(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C)
    annotation (Placement(transformation(extent={{6,-8},{-6,8}},
        rotation=0,
        origin={170,-130})));
  TRANSFORM.Fluid.Sensors.TemperatureTwoPort TC_005(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C, precision
      =3) annotation (Placement(transformation(
        extent={{-10,12},{10,-12}},
        rotation=0,
        origin={196,-170})));
  Fluid.Pipes.NonLinear_Break nonLinear_Break3(redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C)
    annotation (Placement(transformation(extent={{6,-8},{-6,8}},
        rotation=0,
        origin={276,-130})));
  TRANSFORM.Fluid.Valves.ValveLinear PV_008(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    allowFlowReversal=true,
    dp_nominal=100,
    m_flow_nominal=0.840) annotation (Placement(transformation(
        extent={{6,6},{-6,-6}},
        rotation=90,
        origin={342,-142})));
  Modelica.Fluid.Pipes.DynamicPipe pipe8(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    length=7,
    diameter=0.051,
    redeclare model FlowModel =
        Modelica.Fluid.Pipes.BaseClasses.FlowModels.NominalLaminarFlow (
          dp_nominal=10, m_flow_nominal=0.84))
                    annotation (Placement(transformation(
        extent={{7,7},{-7,-7}},
        rotation=0,
        origin={329,-169})));
  Modelica.Blocks.Sources.Constant const2(k=12.6)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={162,-190})));
equation
  connect(sensor_hx_cw.port_b,boundary. ports[1])
    annotation (Line(points={{-28,-80},{12,-80}},
                                               color={0,127,255}));
  connect(hx.port_a2,sensor_hx_cw. port_a) annotation (Line(points={{-58,-98},{
          -52,-98},{-52,-80},{-48,-80}},
                                  color={0,127,255}));
  connect(sensor_cw_hx.port_b,hx. port_b2) annotation (Line(points={{-92,-78},{
          -84,-78},{-84,-98},{-78,-98}},
                                  color={0,127,255}));
  connect(sensor_rp_hx_2.port_b,hx. port_b1) annotation (Line(points={{-44,-118},
          {-52,-118},{-52,-106},{-58,-106}},
                                          color={0,127,255}));
  connect(hx.port_a1,sensor_hx_co. port_a) annotation (Line(points={{-78,-106},
          {-84,-106},{-84,-118},{-90,-118}},
                                         color={0,127,255}));
  connect(sensor_hx_co.port_b,valve_ps. port_b)
    annotation (Line(points={{-110,-118},{-172,-118}},
                                                    color={0,127,255}));
  connect(opening_valve_tank.y,valve_ps. opening) annotation (Line(points={{-205,
          -98},{-182,-98},{-182,-110}},color={0,0,127}));
  connect(pT_co_rp_1.port_b, rp.port_a2) annotation (Line(points={{-36,-40},{
          -48,-40},{-48,-26},{-56,-26}},  color={0,127,255}));
  connect(pT_vc_pipe_rp.port_b, rp.port_a1) annotation (Line(points={{-88,-2},{
          -84,-2},{-84,-18},{-76,-18}},         color={0,127,255}));
  connect(rp.port_b1, pT_rp_hx_1.port_a) annotation (Line(points={{-56,-18},{
          -48,-18},{-48,-2},{-36,-2}},       color={0,127,255}));
  connect(rp.port_b2, pT_rp_pipe_vc.port_a) annotation (Line(points={{-76,-26},
          {-86,-26},{-86,-40}},                 color={0,127,255}));
  connect(pT_pipe_vc.port_b, vc.port_a) annotation (Line(points={{-208,-40},{
          -286,-40},{-286,-38}},    color={0,127,255}));
  connect(vc.port_b,resistance. port_a)
    annotation (Line(points={{-286,-26},{-286,-21}},
                                                   color={0,127,255}));
  connect(pT_vc_pipe.port_a, resistance.port_b) annotation (Line(points={{-268,-2},
          {-286,-2},{-286,-7}},                color={0,127,255}));
  connect(ps.ports[1],valve_ps. port_a) annotation (Line(points={{-242,-118},{
          -192,-118}},                 color={0,127,255}));
  connect(pipe_hx_co.port_b,volume_co. port_a)
    annotation (Line(points={{-122,-182},{-94,-182}},
                                                   color={0,127,255}));
  connect(pipe_hx_co.port_a,valve_ps. port_b) annotation (Line(points={{-142,
          -182},{-160,-182},{-160,-118},{-172,-118}},
                                                  color={0,127,255}));
  connect(mflow_MAGNET.port_b, pipe_co_rp.port_a)
    annotation (Line(points={{-4,-182},{42,-182},{42,-40},{12,-40}},
                                                     color={0,127,255}));
  connect(pipe_co_rp.port_b, pT_co_rp_1.port_a) annotation (Line(points={{-8,-40},
          {-16,-40}},                                  color={0,127,255}));
  connect(volume_co.port_b,co. port_a)
    annotation (Line(points={{-82,-182},{-64,-182}},
                                                   color={0,127,255}));
  connect(co.port_b, mflow_MAGNET.port_a)
    annotation (Line(points={{-44,-182},{-24,-182}},color={0,127,255}));
  connect(pipe_vc_TEDS.port_a, pT_vc_pipe.port_b)
    annotation (Line(points={{-240,-2},{-248,-2}},     color={0,127,255}));
  connect(pT_pipe_vc.port_a, pipe_ins_rp_vc.port_b)
    annotation (Line(points={{-188,-40},{-172,-40}}, color={0,127,255}));
  connect(pT_rp_pipe_vc.port_b, pipe_ins_rp_vc.port_a)
    annotation (Line(points={{-106,-40},{-152,-40}},
                                                   color={0,127,255}));
  connect(sensor_rp_hx_2.port_a,pipe_ins_rp_hx. port_b)
    annotation (Line(points={{-24,-118},{2,-118}},
                                                color={0,127,255}));
  connect(pipe_ins_rp_hx.port_a, pT_rp_hx_1.port_b) annotation (Line(points={{22,-118},
          {50,-118},{50,-2},{-16,-2}},                  color={0,127,255}));
  connect(T_vc_TEDS.port_b, m_flow_vc_TEDS.port_a)
    annotation (Line(points={{-204,66},{-204,74}}, color={0,127,255}));
  connect(T_vc_TEDS.port_a, valve_vc_TEDS.port_b) annotation (Line(points={{-204,46},
          {-204,24}},                                  color={0,127,255}));
  connect(valve_vc_TEDS.port_a, pipe_vc_TEDS.port_b) annotation (Line(points={{-204,12},
          {-204,-2},{-220,-2}},         color={0,127,255}));
  connect(valve_TEDS_rp.port_b, pT_TEDS_rp.port_a) annotation (Line(points={{-140,64},
          {-140,58}},                               color={0,127,255}));
  connect(pipe_vc_rp.port_b, pT_vc_pipe_rp.port_a)
    annotation (Line(points={{-118,-2},{-108,-2}}, color={0,127,255}));
  connect(mflow_cw.port_a, m_source_cw.ports[1])
    annotation (Line(points={{-150,-78},{-174,-78}}, color={0,127,255}));
  connect(mflow_cw.port_b, sensor_cw_hx.port_a)
    annotation (Line(points={{-130,-78},{-112,-78}},
                                                   color={0,127,255}));
  connect(volume_MT.port_a, pT_TEDS_rp.port_b) annotation (Line(points={{-140,24},
          {-140,38}},
        color={0,127,255}));
  connect(mflow_vc_GT.port_b, turbine.inlet) annotation (Line(points={{-340,94},
          {-340.6,94},{-340.6,98.8}},   color={0,127,255}));
  connect(valve_vc_GT.port_a, pipe_vc_TEDS.port_b) annotation (Line(points={{-340,30},
          {-340,8},{-204,8},{-204,-2},{-220,-2}},       color={0,127,255}));
  connect(volume_MT.port_b, pipe_vc_rp.port_a) annotation (Line(points={{-140,12},
          {-140,-2},{-138,-2}},                             color={0,127,255}));
  connect(turbine.outlet, pT_GT_co.port_a) annotation (Line(points={{-340.6,
          125.2},{-340.6,134},{-340,134},{-340,138}},
                                        color={0,127,255}));
  connect(valve_vc_GT.port_b, pT_vc_GT.port_a)
    annotation (Line(points={{-340,42},{-340,48}},     color={0,127,255}));
  connect(pT_vc_GT.port_b, mflow_vc_GT.port_a)
    annotation (Line(points={{-340,68},{-340,74}},   color={0,127,255}));
  connect(compressor.outlet, pT_co_rp.port_a) annotation (Line(points={{-257.2,
          171.6},{-250.6,171.6},{-250.6,172},{-250,172}},
                                                  color={0,127,255}));
  connect(pT_co_rp1.port_b, compressor.inlet) annotation (Line(points={{-290,
          172},{-284.4,172},{-284.4,171.6},{-278.8,171.6}},
                                      color={0,127,255}));
  connect(springBallValve.port_b,boundary5. ports[1])
    annotation (Line(points={{-318,194},{-318,200},{-278,200}},
                                                          color={0,127,255}));
  connect(pT_co_rp.port_b, mflow_GT_rp.port_a)
    annotation (Line(points={{-230,172},{-214,172},{-214,164}},
                                                          color={0,127,255}));
  connect(mflow_GT_rp.port_b, pT_TEDS_rp.port_b) annotation (Line(points={{-214,
          144},{-214,32},{-140,32},{-140,38}},                    color={0,127,
          255}));
  connect(pT_GT_co.port_b, Cooler.port_a) annotation (Line(points={{-340,158},{
          -340,172},{-338,172}},
                               color={0,127,255}));
  connect(Cooler.port_b, pT_co_rp1.port_a)
    annotation (Line(points={{-326,172},{-310,172}},
                                                   color={0,127,255}));
  connect(springBallValve.port_a, pT_co_rp1.port_a) annotation (Line(points={{-318,
          182},{-318,172},{-310,172}},   color={0,127,255}));
  connect(boundary3.port, Cooler.heatPort)
    annotation (Line(points={{-354,186},{-332,186},{-332,178}},
                                                             color={191,0,0}));
  connect(m_flow_vc_TEDS.port_b, MAGNET_TEDS_simpleHX1.port_a1) annotation (
      Line(points={{-204,94},{-204,120},{70,120},{70,112}},  color={0,127,255}));
  connect(pipe4.port_b, TC_002.port_a)
    annotation (Line(points={{78,-34},{78,-19}},     color={0,127,255}));
  connect(TC_003.port_b, tank1.port_a) annotation (Line(points={{150,108},{183,
          108}},                               color={0,127,255}));
  connect(pipe2.port_b,PV_049. port_a)
    annotation (Line(points={{228,108},{238,108},{238,100}},
                                                       color={0,127,255}));
  connect(PV_049.port_b, m_Chg_In.port_a)
    annotation (Line(points={{238,88},{238,86}},   color={0,127,255}));
  connect(PV_050.port_a,sensor_m_flow2. port_a) annotation (Line(points={{308,100},
          {308,108},{360,108},{360,90}},
                                       color={0,127,255}));
  connect(PV_006.port_b,sensor_m_flow2. port_a)
    annotation (Line(points={{292,108},{360,108},{360,90}},
                                                          color={0,127,255}));

  connect(actuatorSubBus.Valve_6_Opening, PV_004.opening) annotation (Line(
      points={{195,138},{156,138},{156,-100},{102,-100},{102,-125.2}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(PV_050.port_b, m_Dch_Out.port_b)
    annotation (Line(points={{308,88},{308,86}},   color={0,127,255}));
  connect(m_Dch_Out.port_a, T_Dch_Out.port_b)
    annotation (Line(points={{308,66},{308,60}},    color={0,127,255}));
  connect(m_Chg_In.port_b, T_Chg_In.port_a)
    annotation (Line(points={{238,66},{238,60}},    color={0,127,255}));

  connect(sensorSubBus.Charging_flowrate, FM_202.m_flow) annotation (Line(
      points={{163,138},{332,138},{332,-96},{311.6,-96},{311.6,-95}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus.Heater_flowrate, FM_003.m_flow) annotation (Line(
      points={{163,138},{156,138},{156,-100},{123,-100},{123,-127.12}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(pipe2.port_b,BOP_Mass_flow. port_a)
    annotation (Line(points={{228,108},{250,108}},
                                               color={0,127,255}));
  connect(BOP_Mass_flow.port_b,PV_006. port_a)
    annotation (Line(points={{270,108},{280,108}},
                                                 color={0,127,255}));
  connect(sensor_m_flow2.port_b,T_chiller_before. port_a)
    annotation (Line(points={{360,70},{360,60}}, color={0,127,255}));
  connect(T_chiller_before.port_b,pipe7. port_a)
    annotation (Line(points={{360,34},{360,22}}, color={0,127,255}));
  connect(pipe7.port_b,T_chiller_after. port_a) annotation (Line(points={{360,6},
          {360,-20}},                color={0,127,255}));
  connect(MAGNET_TEDS_simpleHX1.port_b1, valve_TEDS_rp.port_a) annotation (Line(
        points={{70,92},{70,84},{-140,84},{-140,76}},
        color={0,127,255}));
  connect(TC_002.port_b, MAGNET_TEDS_simpleHX1.port_a2) annotation (Line(points={{78,5},{
          78,92}},                         color={0,127,255}));
  connect(MAGNET_TEDS_simpleHX1.port_b2, TC_003.port_a)
    annotation (Line(points={{78,112},{78,120},{102,120},{102,108},{124,108}},
                                                   color={0,127,255}));
  connect(sensorSubBus.mflow_inside_MAGNET, mflow_inside_MAGNET.y) annotation (
      Line(
      points={{163,138},{136,138},{136,176},{82.9,176}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus.Tout_vc, Tout_vc.y) annotation (Line(
      points={{163,138},{136,138},{136,165},{82.9,165}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus.Tin_vc, Tin_vc.y) annotation (Line(
      points={{163,138},{136,138},{136,153},{82.9,153}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus.MAGNET_flow, m_flow_vc_TEDS.m_flow) annotation (Line(
      points={{163,138},{-300,138},{-300,84},{-207.6,84}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.MAGNET_valve3_opening, valve_TEDS_rp.opening)
    annotation (Line(
      points={{195,138},{60,138},{60,70},{-135.2,70}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.MAGNET_valve_opening, valve_vc_TEDS.opening)
    annotation (Line(
      points={{195,138},{-300,138},{-300,18},{-208.8,18}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.MAGNET_flow_control, co.inputSignal) annotation (Line(
      points={{195,138},{60,138},{60,-140},{-54,-140},{-54,-175}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus.GT_Power, GT_Power.y) annotation (Line(
      points={{163,138},{136,138},{136,186},{82.9,186}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.CW_control, m_source_cw.m_flow_in) annotation (Line(
      points={{195,138},{-300,138},{-300,-70},{-194,-70}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.MAGNET_valve2_opening, valve_vc_GT.opening)
    annotation (Line(
      points={{195,138},{-300,138},{-300,36},{-335.2,36}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus.Tout_TEDSide, TC_003.T) annotation (Line(
      points={{163,138},{163,126},{137,126},{137,112.32}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus.Heater_Input, MAGNET_heater_input.y) annotation (Line(
      points={{163,138},{136,138},{136,196},{82.9,196}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Pump_Flow, P_001.in_p) annotation (Line(
      points={{195,138},{156,138},{156,-124.16}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus,
    control_System_Therminol_4_element_all_modes_MAGNET_GT_dyn_0_1_HW.sensorBus)
    annotation (Line(
      points={{163,138},{232.956,138},{232.956,162.405}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus,
    control_System_Therminol_4_element_all_modes_MAGNET_GT_dyn_0_1_HW.actuatorBus)
    annotation (Line(
      points={{195,138},{245.611,138},{245.611,162.405}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Valve_2_Opening, PV_006.opening) annotation (Line(
      points={{195,138},{286,138},{286,112.8}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(thermocline_Insulation_An.port_a, TC_201.port_b)
    annotation (Line(points={{274,12},{274,16}},     color={0,127,255}));
  connect(T_Chg_In.port_b, TC_201.port_a) annotation (Line(points={{238,40},{
          238,34},{274,34}},     color={0,127,255}));
  connect(T_Dch_Out.port_a, TC_201.port_a) annotation (Line(points={{308,40},{
          308,34},{274,34}},      color={0,127,255}));
  connect(thermocline_Insulation_An.port_b, TC_202.port_b)
    annotation (Line(points={{274,-20},{274,-22}},   color={0,127,255}));
  connect(T_Dch_In.port_b, TC_202.port_a) annotation (Line(points={{239,-50},{
          239,-40},{274,-40}},   color={0,127,255}));
  connect(T_Chg_Out.port_b, TC_202.port_a) annotation (Line(points={{308,-50},{
          308,-40},{274,-40}},    color={0,127,255}));
  connect(Chiller_Mass_Flow.ports[1], HX_001.port_a_tube) annotation (Line(
        points={{206,-198},{216,-198},{216,-178},{231,-178},{231,-177}}, color=
          {0,127,255}));
  connect(HX_001.port_b_tube, Ethylene_glycol_exit_temperature.port_a)
    annotation (Line(points={{262,-177},{262,-178},{280,-178},{280,-190}},
        color={0,127,255}));
  connect(Ethylene_glycol_exit_temperature.port_b,boundary1. ports[1])
    annotation (Line(points={{304,-190},{320,-190}}, color={0,127,255}));
  connect(HX_001.port_b_shell, nonLinear_Break.port_a) annotation (Line(points={{231,
          -170.1},{232,-170},{228,-170}},       color={0,127,255}));
  connect(TC_006.port_a,nonLinear_Break2. port_a) annotation (Line(points={{178,
          -130},{176,-130}},    color={0,127,255}));
  connect(nonLinear_Break2.port_b, P_001.port_a)
    annotation (Line(points={{164,-130},{164,-130}}, color={0,127,255}));
  connect(PV_009.port_b,TC_006. port_b)
    annotation (Line(points={{250,-130},{202,-130}},
                                                   color={0,127,255}));
  connect(HX_001.port_a_shell, TC_004.port_a) annotation (Line(points={{262,
          -170.1},{264,-169},{271,-169}}, color={0,127,255}));
  connect(FM_002.port_b, TC_004.port_b)
    annotation (Line(points={{295,-169},{289,-169}}, color={0,127,255}));
  connect(nonLinear_Break3.port_b,PV_009. port_a) annotation (Line(points={{270,
          -130},{262,-130}},                      color={0,127,255}));
  connect(pipe3.port_b,nonLinear_Break3. port_a)
    annotation (Line(points={{346,-130},{282,-130}}, color={0,127,255}));
  connect(PV_008.port_a,pipe3. port_b) annotation (Line(points={{342,-136},{342,
          -130},{346,-130}}, color={0,127,255}));
  connect(pipe8.port_b, FM_002.port_a)
    annotation (Line(points={{322,-169},{316,-169}}, color={0,127,255}));
  connect(PV_008.port_b,pipe8. port_a) annotation (Line(points={{342,-148},{342,
          -169},{336,-169}}, color={0,127,255}));
  connect(const2.y, Chiller_Mass_Flow.m_flow_in)
    annotation (Line(points={{166.4,-190},{186,-190}}, color={0,0,127}));
  connect(T_chiller_after.port_b, pipe3.port_a) annotation (Line(points={{360,-44},
          {360,-130},{358,-130}},       color={0,127,255}));
  connect(sensorSubBus.TC006, TC_006.T) annotation (Line(
      points={{163,138},{156,138},{156,-100},{190,-100},{190,-125.68}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.PV012, PV_012.opening) annotation (Line(
      points={{195,138},{156,138},{156,-100},{174,-100},{174,-165.2}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.PV008, PV_008.opening) annotation (Line(
      points={{195,138},{380,138},{380,-142},{346.8,-142}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.PV009, PV_009.opening) annotation (Line(
      points={{195,138},{156,138},{156,-100},{222,-100},{222,-112},{256,-112},{
          256,-125.2}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(TC_005.port_a, PV_012.port_a)
    annotation (Line(points={{186,-170},{180,-170}}, color={0,127,255}));
  connect(TC_005.port_b, nonLinear_Break.port_b)
    annotation (Line(points={{206,-170},{216,-170}}, color={0,127,255}));
  connect(PV_012.port_b, TC_006.port_b) annotation (Line(points={{168,-170},{
          162,-170},{162,-154},{226,-154},{226,-130},{202,-130}}, color={0,127,
          255}));
  connect(T_Chg_Out.port_a, PV_052.port_a)
    annotation (Line(points={{308,-68},{308,-74}},   color={0,127,255}));
  connect(PV_052.port_b, FM_202.port_a)
    annotation (Line(points={{308,-86},{308,-86}},   color={0,127,255}));
  connect(FM_202.port_b, PV_008.port_b) annotation (Line(points={{308,-104},{
          308,-154},{342,-154},{342,-148}}, color={0,127,255}));
  connect(pipe4.port_a, PV_004.port_a) annotation (Line(points={{78,-50},{78,
          -130},{96,-130}},  color={0,127,255}));
  connect(PV_004.port_b, FM_003.port_b)
    annotation (Line(points={{108,-130},{114,-130}}, color={0,127,255}));
  connect(FM_003.port_a, P_001.port_b)
    annotation (Line(points={{132,-130},{148,-130}}, color={0,127,255}));
  connect(PV_051.port_b, P_001.port_b) annotation (Line(points={{238,-106},{238,
          -116},{142,-116},{142,-130},{148,-130}}, color={0,127,255}));
  connect(sensorSubBus.Discharge_FlowRate, FM_201.m_flow) annotation (Line(
      points={{163,138},{156,138},{156,-100},{216,-100},{216,-81},{235.76,-81}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));

  connect(PCU_Elec_Demand_kW.y, PCU_Elec_Demand_W.u)
    annotation (Line(points={{126.7,-19},{135,-19}},   color={0,0,127}));
  connect(TEDS_Heat_Demand_kW.y, TEDS_Heat_Demand_W.u)
    annotation (Line(points={{126.7,9},{135,9}},       color={0,0,127}));
  connect(actuatorSubBus.Valve_4_Opening, PV_049.opening) annotation (Line(
      points={{195,138},{156,138},{156,94},{233.2,94}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Valve_1_Opening, PV_052.opening) annotation (Line(
      points={{195,138},{332,138},{332,-80},{312.8,-80}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Valve_5_Opening, PV_050.opening) annotation (Line(
      points={{195,138},{332,138},{332,94},{312.8,94}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.Valve_3_Opening, PV_051.opening) annotation (Line(
      points={{195,138},{156,138},{156,-100},{233.2,-100}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(T_Dch_In.port_a, FM_201.port_b)
    annotation (Line(points={{239,-69},{239,-72}},   color={0,127,255}));
  connect(PV_051.port_a, FM_201.port_a) annotation (Line(points={{238,-94},{239,
          -94},{239,-90}},       color={0,127,255}));
  connect(pipe2.port_a, tank1.port_b)
    annotation (Line(points={{212,108},{197,108}}, color={0,127,255}));
  annotation (experiment(
      StopTime=86400,
      Interval=10,
      __Dymola_Algorithm="Esdirk45a"),
    Diagram(coordinateSystem(extent={{-380,-220},{380,220}})),
    Icon(coordinateSystem(extent={{-380,-220},{380,220}})));
end TEDS_MAGNET_Integration_GT_0_1_bypass;
