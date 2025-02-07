within NHES.Utilities.FMI_Templates.Adaptors.MSLHeatAdaptors.Examples;
model GenerationOfFMUsCS
  "Example to demonstrate variants to generate FMUs (Functional Mock-up Units)"
  extends Modelica.Icons.Example;

  Modelica.Blocks.Sources.Sine sine1(f=2, amplitude=1000)
    annotation (Placement(transformation(extent={{-94,-80},{-74,-60}})));
  Utilities.FMIs.MSLHeatAdaptors_Utilities_ConductionCSdymola_fmu
                                                              conductor(
    G=1000,
    fmi_NumberOfSteps=1000,
    fmi_rTol=0.0001)
    annotation (Placement(transformation(extent={{20,-80},{40,-60}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor capacitor3a(C=1.1, T(fixed=true, start=293.15))
    annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow heatFlow3
    annotation (Placement(transformation(extent={{-60,-80},{-40,-60}})));
  Modelica.Thermal.HeatTransfer.Components.GeneralHeatFlowToTemperatureAdaptor
    heatFlowToTemperature3a(use_pder=false)
    annotation (Placement(transformation(extent={{-10,-80},{10,-60}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor capacitor3b(C=2.2, T(fixed=true, start=293.15))
    annotation (Placement(transformation(extent={{70,-70},{90,-50}})));
  Modelica.Thermal.HeatTransfer.Components.GeneralHeatFlowToTemperatureAdaptor
    heatFlowToTemperature3b(use_pder=false)
    annotation (Placement(transformation(extent={{70,-80},{50,-60}})));
equation
  connect(heatFlowToTemperature3a.f, conductor.Q_flow1)
    annotation (Line(points={{3,-78},{8,-78},{8,-90},{50,-90},{50,-66.6},
          {42,-66.6}},                          color={0,0,127}));
  connect(conductor.Q_flow2,heatFlowToTemperature3b. f)
    annotation (Line(points={{42,-73.3},{50,-73.3},{50,-78},{57,-78}},
                                                 color={0,0,127}));
  connect(heatFlowToTemperature3a.p, conductor.T1)
    annotation (Line(points={{3,-62},{12,-62},{12,-66.6},{19.6,-66.6}},
                                                color={0,0,127}));
  connect(conductor.T2,heatFlowToTemperature3b. p)
    annotation (Line(points={{19.6,-73.3},{10,-73.3},{10,-54},{52,-54},
          {52,-62},{57,-62}},                    color={0,0,127}));
  connect(sine1.y, heatFlow3.Q_flow) annotation (Line(points={{-73,-70},
          {-60,-70}},           color={0,0,127}));
  connect(heatFlow3.port, capacitor3a.port)
    annotation (Line(points={{-40,-70},{-20,-70}}, color={191,0,0}));
  connect(capacitor3a.port,heatFlowToTemperature3a. heatPort)
    annotation (Line(points={{-20,-70},{-2,-70}}, color={191,0,0}));
  connect(heatFlowToTemperature3b.heatPort, capacitor3b.port)
    annotation (Line(points={{62,-70},{80,-70}}, color={191,0,0}));
  annotation (experiment(Interval=0.001, __Dymola_Algorithm="Dassl"),
                                                      Documentation(info="<html>
<p><b>Note</b>: Copy of the Modelica Standard Library Example File. (Just bringing it down to the NHES system to collect together FMU creation.)</p>
<p>This example demonstrates how to generate an input/output block (e.g. in form of an FMU - <a href=\"https://www.fmi-standard.org\">Functional Mock-up Unit</a>) from various HeatTransfer components. The goal is to export such an input/output block from Modelica and import it in another modeling environment. The essential issue is that before exporting it must be known in which way the component is utilized in the target environment. Depending on the target usage, different flange variables need to be in the interface with either input or output causality. Note, this example model can be used to test the FMU export/import of a Modelica tool. Just export the components marked in the icons as &quot;toFMU&quot; as FMUs and import them back. The models should then still work and give the same results as a pure Modelica model. </p>
<p><b>Connecting two masses</b></p><p>The upper part (DirectCapacity, InverseCapacity) demonstrates how to export two heat capacitors and connect them together in a target system. This requires that one of the capacitors (here: DirectCapacity) is defined to have states and the temperature and derivative of temperature are provided in the interface. The other capacitor (here: InverseCapacity) requires heat flow according to the provided input temperature and derivative of temperature. </p>
<p><b>Connecting a conduction element that needs only temperature</b></p><p>The lower part (Conductor) demonstrates how to export a conduction element that needs only temperatures for its conduction law and connect this conduction law in a target system between two capacitors. </p>
</html>"));
end GenerationOfFMUsCS;
