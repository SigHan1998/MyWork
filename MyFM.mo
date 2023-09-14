within ;
package MyFM

  package Tutorial1

    model SimplePendulum
      constant Real g = 9.81;
      parameter Real L = 1;
      Real Theta;
      Real ThetaDot;
    equation
      der(ThetaDot) = -g/L*sin(Theta);
      der(Theta) = ThetaDot;

      annotation (experiment(StopTime=100));
    end SimplePendulum;

    model SimplePendulumIni
      constant Real g = 9.81;
      parameter Real L = 1;
      Real Theta(start = 0.1, fixed = true);
      Real ThetaDot;
    equation
      der(ThetaDot) = -g/L*sin(Theta);
      der(Theta) = ThetaDot;

      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=5000));
    end SimplePendulumIni;
  end Tutorial1;

  package Tutorial2
    model SimplePendulumUnits
      constant Real g(unit="m/s2") = 9.81;
      parameter Real L(min=0, unit="m") = 1;
      Real Theta(start = 0.1, fixed = true);
      Real ThetaDot;
    equation
      der(ThetaDot) = -g/L*sin(Theta);
      der(Theta) = ThetaDot;

      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=5000));
    end SimplePendulumUnits;

    model SimplePendulumSI
      constant Modelica.Units.SI.Acceleration g = 9.81;
      parameter Modelica.Units.SI.Length L(min=0) = 1;
      Modelica.Units.SI.Angle Theta(start = 0.1, fixed = true);
       Modelica.Units.SI.AngularVelocity ThetaDot;
    equation
      der(ThetaDot) = -g/L*sin(Theta);
      der(Theta) = ThetaDot;

      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=5000));
    end SimplePendulumSI;

    model SimplePendulumImport
      import Modelica.Units.SI;
      constant SI.Acceleration g = 9.81;
      parameter SI.Length L(min=0) = 1;
      SI.Angle Theta(start = 0.1, fixed = true);
      SI.AngularVelocity ThetaDot;
    equation
      der(ThetaDot) = -g/L*sin(Theta);
      der(Theta) = ThetaDot;

      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=5000));
    end SimplePendulumImport;

    model SimplePendulumDescription "Model of a simple pendulum"
      import Modelica.Units.SI "Import to allow compact names";
      constant SI.Acceleration g = 9.81 "Gravitational constant";
      parameter SI.Length L(min=0) = 1 "Length of the pendulum";
      /* List of variables */

      SI.Angle Theta(start = 0.1, fixed = true) "Displacement angle";
      SI.AngularVelocity ThetaDot "Displacement velocity";
    equation
      der(Theta) = ThetaDot "Equation to allow second derivative";
      der(ThetaDot) = -g/L*sin(Theta);


      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=5000));
    end SimplePendulumDescription;

    model Motor
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
        annotation (Placement(transformation(extent={{-10,20},{10,40}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
        annotation (Placement(transformation(extent={{40,20},{60,40}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-80,-70},{-60,-50}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-70,0})));
      Modelica.Electrical.Analog.Basic.RotationalEMF emf
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      Modelica.Blocks.Interfaces.RealInput realInput annotation (Placement(
            transformation(extent={{-156,-8},{-140,8}}), iconTransformation(
              extent={{-156,-8},{-140,8}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b1
                        "Flange of right shaft"
        annotation (Placement(transformation(extent={{130,-10},{150,10}})));
    equation
      connect(resistor.n, inductor.p)
        annotation (Line(points={{10,30},{40,30}}, color={0,0,255}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{60,30},{70,30},{70,10}}, color={0,0,255}));
      connect(signalVoltage.n, ground.p)
        annotation (Line(points={{-70,-10},{-70,-50}}, color={0,0,255}));
      connect(signalVoltage.p, resistor.p) annotation (Line(points={{-70,10},{
              -70,30},{-10,30}}, color={0,0,255}));
      connect(emf.n, ground.p) annotation (Line(points={{70,-10},{70,-29},{-70,
              -29},{-70,-50}}, color={0,0,255}));
      connect(emf.flange, inertia.flange_a)
        annotation (Line(points={{80,0},{100,0}}, color={0,0,0}));
      connect(signalVoltage.v, realInput)
        annotation (Line(points={{-82,0},{-148,0}}, color={0,0,127}));
      connect(inertia.flange_b, flange_b1)
        annotation (Line(points={{120,0},{140,0}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -140,-100},{140,100}}), graphics={
            Rectangle(
              extent={{-94,68},{102,-48}},
              lineColor={28,108,200},
              fillColor={238,46,47},
              fillPattern=FillPattern.HorizontalCylinder),
            Polygon(
              points={{-138,-78},{-138,-60},{-84,-58},{-70,-26},{86,-26},{98,
                  -60},{140,-60},{140,-78},{-138,-78}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={0,0,0}),
            Rectangle(
              extent={{102,4},{138,-8}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={0,0,0}),
            Rectangle(
              extent={{-140,2},{-94,-4}},
              lineColor={28,108,200},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={0,0,0})}), Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-140,-100},{140,100}})));
    end Motor;

    model MotorDrive
      Motor motor
        annotation (Placement(transformation(extent={{-14,-10},{14,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{50,-10},{70,10}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{20,-10},{40,10}})));
      Modelica.Blocks.Continuous.PID PID
        annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
      Modelica.Blocks.Sources.Step step
        annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
      Modelica.Blocks.Math.Feedback feedback
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
         Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={90,-30})));
    equation
      connect(motor.flange_b1, idealGear.flange_a)
        annotation (Line(points={{14,0},{20,0}}, color={0,0,0}));
      connect(inertia.flange_a, idealGear.flange_b)
        annotation (Line(points={{50,0},{40,0}}, color={0,0,0}));
      connect(motor.realInput, PID.y)
        annotation (Line(points={{-14.8,0},{-19,0}}, color={0,0,127}));
      connect(PID.u, feedback.y)
        annotation (Line(points={{-42,0},{-51,0}}, color={0,0,127}));
      connect(step.y, feedback.u1)
        annotation (Line(points={{-69,0},{-68,0}}, color={0,0,127}));
      connect(inertia.flange_b, angleSensor.flange)
        annotation (Line(points={{70,0},{90,0},{90,-20}}, color={0,0,0}));
      connect(angleSensor.phi, feedback.u2) annotation (Line(points={{90,-41},{
              90,-52},{-60,-52},{-60,-8}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=100, __Dymola_NumberOfIntervals=5000));
    end MotorDrive;
  end Tutorial2;

  package Tut3
    package Componentes
      model Machine

        parameter Modelica.Units.SI.Resistance R=0.5
          "Resistance of the armature" annotation (Dialog(group="Electrical"));
        parameter Modelica.Units.SI.Inductance L=0.05
          "Inductance of the machine" annotation (Dialog(group="Electrical"));
        parameter Modelica.Units.SI.Inertia J=0.001 "Inertia of the machine"
          annotation (Dialog(tab="Mechanical"));

        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
          annotation (Placement(transformation(extent={{-10,20},{10,40}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)
          annotation (Placement(transformation(extent={{40,20},{60,40}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J)
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        Modelica.Electrical.Analog.Basic.RotationalEMF emf
          annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
          "Flange of right shaft"
          annotation (Placement(transformation(extent={{130,-10},{150,10}})));

        Modelica.Electrical.Analog.Interfaces.PositivePin p1
                      "Positive electrical pin"
          annotation (Placement(transformation(extent={{-150,32},{-130,52}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                                 "Negative electrical pin" annotation (
            Placement(transformation(extent={{-150,-68},{-130,-48}})));
      equation
        connect(resistor.n, inductor.p)
          annotation (Line(points={{10,30},{40,30}}, color={0,0,255}));
        connect(inductor.n, emf.p)
          annotation (Line(points={{60,30},{70,30},{70,10}}, color={0,0,255}));
        connect(emf.flange, inertia.flange_a)
          annotation (Line(points={{80,0},{100,0}}, color={0,0,0}));
        connect(inertia.flange_b, flange)
          annotation (Line(points={{120,0},{140,0}}, color={0,0,0}));
        connect(resistor.p, p1) annotation (Line(points={{-10,30},{-76,30},{-76,
                42},{-140,42}}, color={0,0,255}));
        connect(emf.n, n1) annotation (Line(points={{70,-10},{-4,-10},{-4,-50},
                {-140,-50},{-140,-58}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -140,-100},{140,100}}), graphics={Bitmap(extent={{-122,-82},{
                    142,88}}, fileName=
                    "modelica://MyFM/../FM3217/Resources/Images/dc-motor.jpg")}),
                                      Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-140,-100},{140,100}})));
      end Machine;

      model Turbine
        parameter Modelica.Units.SI.Inertia J=5 "Inertia of the turbine";
        parameter Modelica.Units.SI.Torque tau=0.5 "Torque supplied by the turbine";
        Modelica.Mechanics.Rotational.Components.Inertia inertia(J=J)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
            tau_constant=tau)
          annotation (Placement(transformation(extent={{50,-10},{30,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a1
                          "Flange of left shaft"
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      equation
        connect(constantTorque.flange, inertia.flange_b)
          annotation (Line(points={{30,0},{10,0}}, color={0,0,0}));
        connect(inertia.flange_a, flange_a1)
          annotation (Line(points={{-10,0},{-100,0}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Bitmap(extent={{-88,-86},{100,76}}, fileName="modelica://MyFM/../FM3217/Resources/Images/Turbine.png")}),
                                                                       Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Turbine;

      model Rload
        parameter Modelica.Units.SI.Resistance RL=1 "Resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=RL)
                                                           annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={2,0})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p1
                      "Positive electrical pin"
          annotation (Placement(transformation(extent={{-10,92},{10,112}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                      "Negative electrical pin"
          annotation (Placement(transformation(extent={{-6,-112},{14,-92}})));
      equation
        connect(resistor.p, p1) annotation (Line(points={{2,10},{2,100},{0,100},{0,102}},
              color={0,0,255}));
        connect(resistor.n, n1)
          annotation (Line(points={{2,-10},{4,-10},{4,-102}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Rload;

      model RCload
        parameter Modelica.Units.SI.Resistance RL=1 "Resistance";
        parameter Modelica.Units.SI.Capacitance CL=1 "Capacitance";
        Modelica.Electrical.Analog.Basic.Resistor resistor2(R=RL/2) annotation
          (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={2,0})));
        Modelica.Electrical.Analog.Interfaces.PositivePin p1
                      "Positive electrical pin"
          annotation (Placement(transformation(extent={{-10,92},{10,112}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n1
                      "Negative electrical pin"
          annotation (Placement(transformation(extent={{-6,-112},{14,-92}})));
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=CL)
                                                             annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={56,0})));
        Modelica.Electrical.Analog.Basic.Resistor resistor1(R=RL/2)
                                                           annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={2,36})));
      equation
        connect(resistor2.n, n1) annotation (Line(points={{2,-10},{4,-10},{4,-102}},
              color={0,0,255}));
        connect(capacitor.p, p1) annotation (Line(points={{56,10},{30,10},{30,62},{2,62},
                {2,100},{0,100},{0,102}}, color={0,0,255}));
        connect(capacitor.n, n1) annotation (Line(points={{56,-10},{30,-10},{30,-48},{
                4,-48},{4,-102}}, color={0,0,255}));
        connect(resistor2.p, resistor1.n)
          annotation (Line(points={{2,10},{2,26}}, color={0,0,255}));
        connect(resistor1.p, p1) annotation (Line(points={{2,46},{2,100},{0,100},
                {0,102}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end RCload;

      model RLCload
        extends RCload;
        parameter Modelica.Units.SI.Inductance Ll=0.5 "Inductive part of the load";
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=Ll)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-54,-2})));
      equation
        connect(inductor.p, p1) annotation (Line(points={{-54,8},{-54,60},{2,60},
                {2,100},{0,100},{0,102}}, color={0,0,255}));
        connect(inductor.n, n1) annotation (Line(points={{-54,-12},{-54,-50},{4,
                -50},{4,-102}}, color={0,0,255}));
      end RLCload;
    end Componentes;

    package Tests
      extends Modelica.Icons.ExamplesPackage;
      model MachineTest
        extends Modelica.Icons.ExamplesPackage;
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=10)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-66,4})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-66,-68},{-46,-48}})));
        Modelica.Mechanics.Rotational.Components.Inertia inertia
          annotation (Placement(transformation(extent={{54,-10},{74,10}})));
        Componentes.Machine machine(R=1)
          annotation (Placement(transformation(extent={{-14,-8},{14,12}})));
      equation
        connect(constantVoltage.p, machine.p1) annotation (Line(points={{-66,14},{-40,
                14},{-40,8},{-14,8},{-14,6.2}}, color={0,0,255}));
        connect(ground.p, constantVoltage.n) annotation (Line(points={{-56,-48},{-62,
                -48},{-62,-6},{-66,-6}}, color={0,0,255}));
        connect(machine.n1, constantVoltage.n) annotation (Line(points={{-14,-3.8},{
                -40,-3.8},{-40,-6},{-66,-6}}, color={0,0,255}));
        connect(machine.flange, inertia.flange_a)
          annotation (Line(points={{14,2},{34,2},{34,0},{54,0}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end MachineTest;

      model TurbineTest
        extends Modelica.Icons.ExamplesPackage;
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-66,4})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-66,-68},{-46,-48}})));
        Componentes.Machine machine(R=0.5)
          annotation (Placement(transformation(extent={{-14,-8},{14,12}})));
        Componentes.Turbine turbine
          annotation (Placement(transformation(extent={{42,-8},{62,12}})));
        Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
          annotation (Placement(transformation(extent={{-42,22},{-30,34}})));
      equation
        connect(ground.p, constantVoltage.n) annotation (Line(points={{-56,-48},{-62,
                -48},{-62,-6},{-66,-6}}, color={0,0,255}));
        connect(machine.n1, constantVoltage.n) annotation (Line(points={{-14,-3.8},{
                -40,-3.8},{-40,-6},{-66,-6}}, color={0,0,255}));
        connect(machine.flange, turbine.flange_a1)
          annotation (Line(points={{14,2},{42,2}}, color={0,0,0}));
        connect(constantVoltage.p, powerSensor.pc) annotation (Line(points={{-66,14},
                {-60,14},{-60,28},{-42,28}}, color={0,0,255}));
        connect(powerSensor.nc, machine.p1)
          annotation (Line(points={{-30,28},{-14,28},{-14,6.2}}, color={0,0,255}));
        connect(powerSensor.pv, powerSensor.pc) annotation (Line(points={{-36,34},{-48,
                34},{-48,38},{-52,38},{-52,28},{-42,28}}, color={0,0,255}));
        connect(powerSensor.nv, constantVoltage.n) annotation (Line(points={{-36,22},
                {-36,-4},{-34,-4},{-34,-3.8},{-40,-3.8},{-40,-6},{-66,-6}}, color={0,
                0,255}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=10, __Dymola_NumberOfIntervals=1000));
      end TurbineTest;

      model LoadTest
        extends Modelica.Icons.ExamplesPackage;
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{8,-68},{28,-48}})));
        Componentes.Machine machine(R=0.5)
          annotation (Placement(transformation(extent={{18,-8},{46,12}})));
        Componentes.Turbine turbine(tau=10)
          annotation (Placement(transformation(extent={{72,-10},{92,10}})));
        Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
          annotation (Placement(transformation(extent={{-10,22},{2,34}})));
        Componentes.Rload rload
          annotation (Placement(transformation(extent={{-44,-4},{-24,16}})));
        Componentes.RCload rCload
          annotation (Placement(transformation(extent={{-70,-4},{-50,16}})));
        Componentes.RLCload rLCload
          annotation (Placement(transformation(extent={{-98,-4},{-78,16}})));
      equation
        connect(machine.flange, turbine.flange_a1)
          annotation (Line(points={{46,2},{60,2},{60,0},{72,0}}, color={0,0,0}));
        connect(powerSensor.nc, machine.p1)
          annotation (Line(points={{2,28},{18,28},{18,6.2}}, color={0,0,255}));
        connect(powerSensor.pv, powerSensor.pc) annotation (Line(points={{-4,34},{-16,
                34},{-16,38},{-20,38},{-20,28},{-10,28}}, color={0,0,255}));
        connect(rload.p1, powerSensor.pc) annotation (Line(points={{-34,16.2},{-32,16.2},
                {-32,16},{-28,16},{-28,28},{-10,28}}, color={0,0,255}));
        connect(rload.n1, machine.n1) annotation (Line(points={{-33.6,-4.2},{-33.6,-20},
                {18,-20},{18,-3.8}}, color={0,0,255}));
        connect(ground.p, machine.n1)
          annotation (Line(points={{18,-48},{18,-3.8}}, color={0,0,255}));
        connect(rCload.n1, machine.n1) annotation (Line(points={{-59.6,-4.2},{-59.6,
                -20},{18,-20},{18,-3.8}}, color={0,0,255}));
        connect(rCload.p1, rload.p1) annotation (Line(points={{-60,16.2},{-60,30},{-34,
                30},{-34,16.2}}, color={0,0,255}));
        connect(rLCload.p1, rload.p1) annotation (Line(points={{-88,16.2},{-74,16.2},
                {-74,30},{-34,30},{-34,16.2}}, color={0,0,255}));
        connect(rLCload.n1, machine.n1) annotation (Line(points={{-87.6,-4.2},{-87.6,
                -20},{18,-20},{18,-3.8}}, color={0,0,255}));
        connect(powerSensor.nv, machine.n1) annotation (Line(points={{-4,22},{-4,-20},
                {18,-20},{18,-3.8}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end LoadTest;

      model LoadTest20
        extends LoadTest(turbine(tau=20));
      end LoadTest20;

      model LoadTest30
        extends LoadTest(turbine(tau=30));
      end LoadTest30;

      model LoadTest40 "DUPLICATEEE"
        extends Modelica.Icons.ExamplesPackage;
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{8,-68},{28,-48}})));
        Componentes.Machine machine(R=0.5)
          annotation (Placement(transformation(extent={{18,-8},{46,12}})));
        Componentes.Turbine turbine(tau=40)
          annotation (Placement(transformation(extent={{72,-10},{92,10}})));
        Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
          annotation (Placement(transformation(extent={{-10,22},{2,34}})));
        Componentes.Rload rload
          annotation (Placement(transformation(extent={{-44,-4},{-24,16}})));
        Componentes.RCload rCload
          annotation (Placement(transformation(extent={{-70,-4},{-50,16}})));
        Componentes.RLCload rLCload
          annotation (Placement(transformation(extent={{-98,-4},{-78,16}})));
      equation
        connect(machine.flange, turbine.flange_a1)
          annotation (Line(points={{46,2},{60,2},{60,0},{72,0}}, color={0,0,0}));
        connect(powerSensor.nc, machine.p1)
          annotation (Line(points={{2,28},{18,28},{18,6.2}}, color={0,0,255}));
        connect(powerSensor.pv, powerSensor.pc) annotation (Line(points={{-4,34},{-16,
                34},{-16,38},{-20,38},{-20,28},{-10,28}}, color={0,0,255}));
        connect(rload.p1, powerSensor.pc) annotation (Line(points={{-34,16.2},{-32,16.2},
                {-32,16},{-28,16},{-28,28},{-10,28}}, color={0,0,255}));
        connect(rload.n1, machine.n1) annotation (Line(points={{-33.6,-4.2},{-33.6,-20},
                {18,-20},{18,-3.8}}, color={0,0,255}));
        connect(ground.p, machine.n1)
          annotation (Line(points={{18,-48},{18,-3.8}}, color={0,0,255}));
        connect(rCload.n1, machine.n1) annotation (Line(points={{-59.6,-4.2},{-59.6,
                -20},{18,-20},{18,-3.8}}, color={0,0,255}));
        connect(rCload.p1, rload.p1) annotation (Line(points={{-60,16.2},{-60,30},{-34,
                30},{-34,16.2}}, color={0,0,255}));
        connect(rLCload.p1, rload.p1) annotation (Line(points={{-88,16.2},{-74,16.2},
                {-74,30},{-34,30},{-34,16.2}}, color={0,0,255}));
        connect(rLCload.n1, machine.n1) annotation (Line(points={{-87.6,-4.2},{-87.6,
                -20},{18,-20},{18,-3.8}}, color={0,0,255}));
        connect(powerSensor.nv, machine.n1) annotation (Line(points={{-4,22},{-4,-20},
                {18,-20},{18,-3.8}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end LoadTest40;
    end Tests;
    annotation ();
  end Tut3;
  annotation (uses(Modelica(version="4.0.0")));
end MyFM;
