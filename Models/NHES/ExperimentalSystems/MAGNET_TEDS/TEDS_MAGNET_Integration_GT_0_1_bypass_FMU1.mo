within NHES.ExperimentalSystems.MAGNET_TEDS;
model TEDS_MAGNET_Integration_GT_0_1_bypass_FMU1
  "Dynamic simulation of the integrated system of MAGNET, TEDS, and Gas turbine with a central control system"
  extends TRANSFORM.Icons.Example;

protected
  inner TRANSFORM.Fluid.SystemTF systemTF(
    showColors=true,
    val_min=data.T_hx_co,
    val_max=data.T_vc_rp)
    annotation (Placement(transformation(extent={{-118,-40},{-98,-20}})));

protected
  inner TRANSFORM.Fluid.System system(
    p_ambient=18000,
    T_ambient=498.15,
    m_flow_start=0.84)
    annotation (Placement(transformation(extent={{-98,-40},{-78,-20}})));
protected
  NHES.ExperimentalSystems.MAGNET.Data.Data_base_An data(eta_mech=0.4)
    annotation (Placement(transformation(extent={{-138,-40},{-118,-20}})));
protected
  package Medium = Modelica.Media.IdealGases.SingleGases.N2;//TRANSFORM.Media.ExternalMedia.CoolProp.Nitrogen;
  package Medium_cw = Modelica.Media.Water.StandardWater;
  package Medium_TEDS =
      TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C;

public
  Systems.Experiments.TEDS.BaseClasses.SignalSubBus_ActuatorInput sensorSubBus
    annotation (Placement(transformation(extent={{16,40},{38,64}})));
  Systems.Experiments.TEDS.BaseClasses.SignalSubBus_SensorOutput actuatorSubBus
    annotation (Placement(transformation(extent={{48,40},{70,64}})));
  Modelica.Blocks.Sources.RealExpression MAGNET_heater_input(y=Q_MT_HX_W.y)
    annotation (Placement(transformation(extent={{-22,44},{-4,60}})));
  Magnet_TEDS.MAGNET_TEDS_ControlSystem.Control_System_Therminol_4_element_all_modes_MAGNET_GT_dyn_0_1_bypass_FMU1
    control_System_Therminol_4_element_all_modes_MAGNET_GT_dyn_0_1_HW(
    redeclare package Medium =
        TRANSFORM.Media.Fluids.Therminol_66.LinearTherminol66_A_250C,
    TEDS_Heat_Load_Demand=TEDS_Heat_Demand_W.y,
    T_hot_design=598.15,
    T_cold_design=498.15)
    annotation (Placement(transformation(extent={{24,92},{58,126}})));
  Modelica.Blocks.Interfaces.RealOutput PCU_Elec_Demand_kW_output annotation (
      Placement(transformation(extent={{120,94},{144,118}}),
        iconTransformation(extent={{128,68},{152,92}})));
  Modelica.Blocks.Interfaces.RealOutput m_tes_charged_sp_output annotation (
      Placement(transformation(extent={{120,74},{144,98}}), iconTransformation(
          extent={{128,68},{152,92}})));
  Modelica.Blocks.Interfaces.RealOutput m_tes_discharged_sp_output annotation (
      Placement(transformation(extent={{120,54},{144,78}}), iconTransformation(
          extent={{128,68},{152,92}})));
  Modelica.Blocks.Sources.Pulse Pulse_TEDS_Heat_Demand_kW(
    amplitude=15,
    period=7200,
    offset=35.59071,
    startTime=10800)
    annotation (Placement(transformation(extent={{-122,56},{-108,70}})));
  Modelica.Blocks.Math.Add add_heat
    annotation (Placement(transformation(extent={{-98,60},{-78,80}})));
  Modelica.Blocks.Math.Gain TEDS_Heat_Demand_W(k=1000)
    annotation (Placement(transformation(extent={{-62,64},{-50,76}})));
  Modelica.Blocks.Interfaces.RealInput PCU_Elec_Demand_kW_input annotation (
      Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={-136,116}),iconTransformation(extent={{-120,48},{-96,72}})));
  Modelica.Blocks.Interfaces.RealInput TEDS_Heat_Demand_kW_input annotation (
      Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={-136,76}),iconTransformation(extent={{-120,48},{-96,72}})));
  Modelica.Blocks.Math.Add add_Q_MT_HX
    annotation (Placement(transformation(extent={{-98,18},{-78,38}})));
  Modelica.Blocks.Sources.Pulse pulse_Q_MT_HX_W(
    amplitude=28000,
    period=7200,
    offset=23947,
    startTime=10800)
    annotation (Placement(transformation(extent={{-122,14},{-108,28}})));
  Modelica.Blocks.Math.Gain Q_MT_HX_W(k=1)
    annotation (Placement(transformation(extent={{-62,22},{-50,34}})));
  Modelica.Blocks.Interfaces.RealInput Q_MT_HX_W_input annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=0,
        origin={-136,34}),iconTransformation(extent={{-120,48},{-96,72}})));
  TRANSFORM.Blocks.RealExpression m_tes_charged_sp "Used in the Coded Section"
    annotation (Placement(transformation(extent={{104,30},{128,52}})));
  TRANSFORM.Blocks.RealExpression m_tes_discharged_sp
    "Used in the Coded Section"
    annotation (Placement(transformation(extent={{104,12},{128,34}})));
equation
  PCU_Elec_Demand_kW_output =PCU_Elec_Demand_kW_input;
  m_tes_charged_sp_output = m_tes_charged_sp.y;
  m_tes_discharged_sp_output = m_tes_discharged_sp.y;


  connect(sensorSubBus.Heater_Input, MAGNET_heater_input.y) annotation (Line(
      points={{27,52},{-3.1,52}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(sensorSubBus,
    control_System_Therminol_4_element_all_modes_MAGNET_GT_dyn_0_1_HW.sensorBus)
    annotation (Line(
      points={{27,52},{28,52},{28,86},{34.9556,86},{34.9556,92.4048}},
      color={111,216,99},
      pattern=LinePattern.Dash,
      thickness=0.5));

  connect(TEDS_Heat_Demand_kW_input, add_heat.u1)
    annotation (Line(points={{-136,76},{-100,76}},
                                                 color={0,0,127}));
  connect(add_heat.y, TEDS_Heat_Demand_W.u)
    annotation (Line(points={{-77,70},{-63.2,70}}, color={0,0,127}));
  connect(Q_MT_HX_W_input, add_Q_MT_HX.u1)
    annotation (Line(points={{-136,34},{-100,34}},
                                                 color={0,0,127}));
  connect(add_Q_MT_HX.u2, pulse_Q_MT_HX_W.y) annotation (Line(points={{-100,22},
          {-107.3,22},{-107.3,21}},
                                 color={0,0,127}));
  connect(add_Q_MT_HX.y, Q_MT_HX_W.u)
    annotation (Line(points={{-77,28},{-63.2,28}}, color={0,0,127}));
  connect(add_heat.u2, Pulse_TEDS_Heat_Demand_kW.y)
    annotation (Line(points={{-100,64},{-98,63},{-107.3,63}},
                                                            color={0,0,127}));
  connect(actuatorSubBus,
    control_System_Therminol_4_element_all_modes_MAGNET_GT_dyn_0_1_HW.actuatorBus)
    annotation (Line(
      points={{59,52},{60,52},{60,86},{47.6111,86},{47.6111,92.4048}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.m_tes_charged_sp, m_tes_charged_sp.u) annotation (Line(
      points={{59,52},{60,52},{60,41},{101.6,41}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(actuatorSubBus.m_tes_discharged_sp, m_tes_discharged_sp.u)
    annotation (Line(
      points={{59,52},{60,52},{60,23},{101.6,23}},
      color={239,82,82},
      pattern=LinePattern.Dash,
      thickness=0.5));
  annotation (experiment(
      StopTime=86400,
      Interval=10,
      __Dymola_Algorithm="Esdirk45a"),
    Diagram(coordinateSystem(extent={{-180,-80},{180,160}})),
    Icon(coordinateSystem(extent={{-180,-80},{180,160}})));
end TEDS_MAGNET_Integration_GT_0_1_bypass_FMU1;
