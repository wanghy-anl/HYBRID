within NHES.Systems.SupervisoryControl;
model InputSetpointData_TESUseCase_AllES
  extends BaseClasses.Partial_ControlSystem;

  parameter Real delayStart=5000 "Time to start tracking power profiles";
  parameter SI.Time timeScale=60*60 "Time scale of first table column";

  parameter SI.Power W_nominal_BOP = 4.13285e8 "Nominal BOP Power";
  parameter SI.Power W_nominal_ES = 0 "Nominal ES Power";

  output SI.Power W_totalSetpoint_BOP = switch_BOP.y annotation(Dialog(group="Outputs",enable=false));
  output SI.Power W_totalSetpoint_ES = switch_ES.y annotation(Dialog(group="Outputs",enable=false));
  output SI.Power W_totalSetpoint_EG = switch_EG.y annotation(Dialog(group="Outputs",enable=false));

  Modelica.Blocks.Logical.LessThreshold greaterEqualThreshold1(threshold=
        delayStart)
    annotation (Placement(transformation(extent={{-120,20},{-100,40}})));

  Modelica.Blocks.Sources.ContinuousClock clock(offset=0, startTime=0)
    annotation (Placement(transformation(extent={{-160,20},{-140,40}})));

  Modelica.Blocks.Sources.CombiTimeTable demand_EG(
    tableOnFile=true,
    startTime=delayStart,
    timeScale=timeScale,
    tableName="NetDemand",
    fileName=fileName)
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  Modelica.Blocks.Logical.Switch switch_EG
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
  Modelica.Blocks.Sources.Constant nominal_EG(k=nominal_BOP.k + nominal_ES.k)
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  parameter String fileName=Modelica.Utilities.Files.loadResource(
      "modelica://NHES/Resources/Data/RAVEN/timeSeriesData.txt")
    "File where matrix is stored";
  Modelica.Blocks.Sources.CombiTimeTable demand_BOP(
    tableOnFile=true,
    startTime=delayStart,
    tableName="BOP",
    timeScale=timeScale,
    fileName=fileName)
    annotation (Placement(transformation(extent={{-80,70},{-60,90}})));
  Modelica.Blocks.Logical.Switch switch_BOP
    annotation (Placement(transformation(extent={{-40,90},{-20,110}})));
  Modelica.Blocks.Sources.Constant nominal_BOP(k=W_nominal_BOP)
    annotation (Placement(transformation(extent={{-80,110},{-60,130}})));
equation
  connect(clock.y, greaterEqualThreshold1.u)
    annotation (Line(points={{-139,30},{-122,30}},           color={0,0,127}));
  connect(nominal_EG.y, switch_EG.u1) annotation (Line(points={{-59,-30},{-50,
          -30},{-50,-42},{-42,-42}},
                            color={0,0,127}));
  connect(demand_EG.y[1], switch_EG.u3) annotation (Line(points={{-59,-70},{-50,
          -70},{-50,-58},{-42,-58}},
                              color={0,0,127}));
  connect(demand_BOP.y[1],switch_BOP. u3) annotation (Line(points={{-59,80},{
          -48,80},{-48,92},{-42,92}},   color={0,0,127}));
  connect(nominal_BOP.y,switch_BOP. u1) annotation (Line(points={{-59,120},{-48,
          120},{-48,108},{-42,108}}, color={0,0,127}));
  connect(greaterEqualThreshold1.y, switch_BOP.u2) annotation (Line(points={{
          -99,30},{-90,30},{-90,100},{-42,100}}, color={255,0,255}));
  connect(greaterEqualThreshold1.y, switch_EG.u2) annotation (Line(points={{-99,
          30},{-90,30},{-90,-50},{-42,-50}}, color={255,0,255}));
annotation(defaultComponentName="SC", experiment(StopTime=3600,
        __Dymola_NumberOfIntervals=3600),
    Diagram(coordinateSystem(extent={{-160,-100},{160,180}})),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}}), graphics={
                  Text(
          extent={{-94,82},{94,74}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={255,255,237},
          fillPattern=FillPattern.Solid,
          textString="Input Setpoints: Modelica Path")}));
end InputSetpointData_TESUseCase_AllES;
