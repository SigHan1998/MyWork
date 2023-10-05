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
        Modelica.Electrical.Analog.Basic.Resistor resistor2(R=RL/2) annotation (
           Placement(transformation(
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

  package Tut4
    model ElectricKettle
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=230^2/2000,
          useHeatPort=true) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=-90,
            origin={-12,10})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-100,-66},{-80,-46}})));
      Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(V=230*sqrt(2),
          f=50) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-90,6})));
      Modelica.Electrical.Analog.Sensors.PowerSensor powerSensor
        annotation (Placement(transformation(extent={{-58,30},{-38,50}})));
      Modelica.Blocks.Math.Mean mean(f=50) annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=-90,
            origin={-59,3})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Water(C=4.18*1700,
          T(start=283.15, fixed=true))
        annotation (Placement(transformation(extent={{-4,26},{16,46}})));
      Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{20,0},{40,20}})));
      Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch
        annotation (Placement(transformation(extent={{-82,30},{-62,50}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor
        thermalConductor(G=5) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={6,-22})));
      Modelica.Thermal.HeatTransfer.Celsius.FixedTemperature RoomTemperature(T=
            21) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-26,-54})));
      Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=3)
        annotation (Placement(transformation(extent={{56,6},{76,26}})));
      Modelica.Blocks.Sources.Constant const(k=95)
        annotation (Placement(transformation(extent={{20,32},{40,52}})));
    equation
      connect(resistor.n, sineVoltage.n) annotation (Line(points={{-12,0},{-12,
              -18},{-90,-18},{-90,-4}}, color={0,0,255}));
      connect(ground.p, sineVoltage.n)
        annotation (Line(points={{-90,-46},{-90,-4}}, color={0,0,255}));
      connect(powerSensor.nc, resistor.p) annotation (Line(points={{-38,40},{
              -12,40},{-12,20}}, color={0,0,255}));
      connect(powerSensor.nv, sineVoltage.n) annotation (Line(points={{-48,30},
              {-48,-18},{-90,-18},{-90,-4}}, color={0,0,255}));
      connect(powerSensor.pv, resistor.p) annotation (Line(points={{-48,50},{
              -48,62},{-12,62},{-12,20}}, color={0,0,255}));
      connect(powerSensor.power, mean.u)
        annotation (Line(points={{-58,29},{-58,9},{-59,9}}, color={0,0,127}));
      connect(Water.port, resistor.heatPort)
        annotation (Line(points={{6,26},{6,10},{-2,10}}, color={191,0,0}));
      connect(temperatureSensor.port, resistor.heatPort)
        annotation (Line(points={{20,10},{-2,10}}, color={191,0,0}));
      connect(sineVoltage.p, switch.p) annotation (Line(points={{-90,16},{-90,
              40},{-82,40}}, color={0,0,255}));
      connect(switch.n, powerSensor.pc)
        annotation (Line(points={{-62,40},{-58,40}}, color={0,0,255}));
      connect(resistor.heatPort, thermalConductor.port_a)
        annotation (Line(points={{-2,10},{6,10},{6,-12}}, color={191,0,0}));
      connect(thermalConductor.port_b, RoomTemperature.port)
        annotation (Line(points={{6,-32},{6,-54},{-16,-54}}, color={191,0,0}));
      connect(temperatureSensor.T, onOffController.u)
        annotation (Line(points={{40,10},{54,10}}, color={0,0,127}));
      connect(const.y, onOffController.reference) annotation (Line(points={{41,
              42},{48,42},{48,22},{54,22}}, color={0,0,127}));
      connect(onOffController.y, switch.control) annotation (Line(points={{77,
              16},{84,16},{84,80},{-72,80},{-72,52}}, color={255,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=600, __Dymola_NumberOfIntervals=5000));
    end ElectricKettle;

    model TestHeatConductance
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor Water(C=4.18*1700,
          T(start=368.15, fixed=true))
        annotation (Placement(transformation(extent={{-16,42},{4,62}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor
        thermalConductor(G=5) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-6,10})));
      Modelica.Thermal.HeatTransfer.Celsius.FixedTemperature RoomTemperature(T=
            21) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-38,-22})));
    equation
      connect(thermalConductor.port_b, RoomTemperature.port)
        annotation (Line(points={{-6,0},{-6,-22},{-28,-22}}, color={191,0,0}));
      connect(Water.port, thermalConductor.port_a)
        annotation (Line(points={{-6,42},{-6,20}}, color={191,0,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=3600, __Dymola_NumberOfIntervals=5000));
    end TestHeatConductance;
  end Tut4;

  package Tut5
    model ConnectingPipes
      inner HydroPower.System_HPL system_HPL(steadyState=true,
          constantTemperature=true)
        annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
      HydroPower.HydroSystems.Pipe pipe1(L=100, ZL=90)
        annotation (Placement(transformation(extent={{-10,44},{10,64}})));
      HydroPower.SinksAndSources.Fixed_pT source1(paraOption=false)
        annotation (Placement(transformation(extent={{-60,44},{-40,64}})));
      HydroPower.SinksAndSources.Fixed_pT sink1(paraOption=false)
        annotation (Placement(transformation(extent={{62,44},{42,64}})));
    equation
      connect(source1.b, pipe1.a)
        annotation (Line(points={{-39,54},{-11,54}}, color={0,0,255}));
      connect(sink1.b, pipe1.b)
        annotation (Line(points={{41,54},{11,54}}, color={0,0,255}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=100,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end ConnectingPipes;

    model PipeWithValve
      extends ConnectingPipes(system_HPL(pipeRoughness=0));
      HydroPower.HydroSystems.PipeValve pipeValve2(
        m_dot_nom=1000*100e6/(1e3*9.81*90),
        dp_nom=101325*90/9.81,
        d_nom(displayUnit="kg/m3"),
        ZL=90)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      HydroPower.SinksAndSources.Fixed_pT source2(paraOption=false)
        annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
      HydroPower.SinksAndSources.Fixed_pT sink2(paraOption=false)
        annotation (Placement(transformation(extent={{60,-10},{40,10}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-0.9,
        duration=10,
        offset=1,
        startTime=20)
        annotation (Placement(transformation(extent={{-100,18},{-80,38}})));
    equation
      connect(pipeValve2.b, sink2.b)
        annotation (Line(points={{11,0},{39,0}}, color={0,0,255}));
      connect(pipeValve2.a, source2.b)
        annotation (Line(points={{-11,0},{-41,0}}, color={0,0,255}));
      connect(ramp.y, pipeValve2.ValveCtrl)
        annotation (Line(points={{-79,28},{0,28},{0,11}}, color={0,0,127}));
    end PipeWithValve;

    model SimpleWaterWay
      extends PipeWithValve;
      HydroPower.SinksAndSources.Fixed_pT source3(paraOption=false)
        annotation (Placement(transformation(extent={{-82,-50},{-62,-30}})));
      HydroPower.SinksAndSources.Fixed_pT sink3(paraOption=false)
        annotation (Placement(transformation(extent={{84,-50},{64,-30}})));
      HydroPower.HydroSystems.PipeValve pipeValve3(
        m_dot_nom=1000*100e6/(1e3*9.81*90),
        dp_nom=101325*90/9.81,
        d_nom(displayUnit="kg/m3"),
        ZL=90)
        annotation (Placement(transformation(extent={{26,-50},{46,-30}})));
      HydroPower.HydroSystems.Pipe pipe3(
        horizontalIcon=true,
        L=10000,
        ZL=100,
        ZR=90)
        annotation (Placement(transformation(extent={{-38,-50},{-18,-30}})));
      HydroPower.HydroSystems.HydroComponents.Containers.ClosedVolume
        closedVolume(D=10)
        annotation (Placement(transformation(extent={{-8,-50},{12,-30}})));
    equation
      connect(pipeValve3.ValveCtrl, pipeValve2.ValveCtrl) annotation (Line(
            points={{36,-29},{36,-20},{26,-20},{26,28},{0,28},{0,11}}, color={0,
              0,127}));
      connect(sink3.b, pipeValve3.b)
        annotation (Line(points={{63,-40},{47,-40}}, color={0,0,255}));
      connect(pipe3.a, source3.b)
        annotation (Line(points={{-39,-40},{-61,-40}}, color={0,0,255}));
      connect(pipe3.b, closedVolume.a)
        annotation (Line(points={{-17,-40},{-8,-40}}, color={0,0,255}));
      connect(closedVolume.b, pipeValve3.a)
        annotation (Line(points={{12,-40},{25,-40}}, color={0,0,255}));
    end SimpleWaterWay;

    model SimpleWaterWayWithSurgeTank
      extends SimpleWaterWay;
      HydroPower.SinksAndSources.Fixed_pT source4(paraOption=false)
        annotation (Placement(transformation(extent={{-84,-90},{-64,-70}})));
      HydroPower.SinksAndSources.Fixed_pT sink4(paraOption=false)
        annotation (Placement(transformation(extent={{88,-90},{68,-70}})));
      HydroPower.HydroSystems.PipeValve pipeValve4(
        m_dot_nom=1000*100e6/(1e3*9.81*90),
        dp_nom=101325*90/9.81,
        d_nom(displayUnit="kg/m3"),
        ZL=90)
        annotation (Placement(transformation(extent={{26,-90},{46,-70}})));
      HydroPower.HydroSystems.Pipe pipe4(
        horizontalIcon=true,
        L=10000,
        ZL=100,
        ZR=90)
        annotation (Placement(transformation(extent={{-54,-90},{-34,-70}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(
        D=10,
        deltZ=100,
        Vol=100)
        annotation (Placement(transformation(extent={{-16,-90},{4,-70}})));
    equation
      connect(source4.b, pipe4.a)
        annotation (Line(points={{-63,-80},{-55,-80}}, color={0,0,255}));
      connect(pipe4.b, surgeTank.a)
        annotation (Line(points={{-33,-80},{-17,-80}}, color={0,0,255}));
      connect(pipeValve4.a, surgeTank.b)
        annotation (Line(points={{25,-80},{5,-80}}, color={0,0,255}));
      connect(pipeValve4.ValveCtrl, pipeValve2.ValveCtrl) annotation (Line(
            points={{36,-69},{28,-69},{28,-60},{18,-60},{18,28},{0,28},{0,11}},
            color={0,0,127}));
      connect(pipeValve4.b, sink4.b)
        annotation (Line(points={{47,-80},{67,-80}}, color={0,0,255}));
    end SimpleWaterWayWithSurgeTank;
  end Tut5;

  package Tut6
    model ReservoirBase
      inner HydroPower.System_HPL system_HPL(steadyState=true,
          constantTemperature=true)
        annotation (Placement(transformation(extent={{-96,76},{-76,96}})));
      HydroPower.HydroSystems.Reservoir headwater(steadyState=false)
        annotation (Placement(transformation(extent={{-68,36},{-48,56}})));
      HydroPower.HydroSystems.Reservoir tailwater
        annotation (Placement(transformation(extent={{38,24},{58,44}})));
      HydroPower.HydroSystems.Pipe conduit(horizontalIcon=true, L=1000)
        annotation (Placement(transformation(extent={{-42,30},{-22,50}})));
    equation
      connect(headwater.a2_pipe, conduit.a)
        annotation (Line(points={{-47,40},{-43,40}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end ReservoirBase;

    model TwoReservoir
      extends ReservoirBase(conduit(
          horizontalIcon=false,
          L=1000,
          ZL=100));
    equation
      connect(conduit.b, tailwater.a1_pipe) annotation (Line(points={{-21,40},{
              22,40},{22,28},{37,28}}, color={0,0,255}));
      annotation (experiment(
          StopTime=600,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end TwoReservoir;

    model TwoReservoirsWithSource
      extends TwoReservoir;
      HydroPower.SinksAndSources.Fixed_HT ConstantHeadLevel(
        paraOption=false,
        H_const=headwater.H_start[1],
        Hmax=headwater.Hmax[1],
        depth=headwater.depth[1])
        annotation (Placement(transformation(extent={{-94,48},{-74,68}})));
      HydroPower.SinksAndSources.Fixed_HT fixedTailwaterLevel(
        paraOption=false,
        H_const=tailwater.H_start[tailwater.n],
        Hmax=tailwater.Hmax[tailwater.n],
        depth=tailwater.depth[tailwater.n])
        annotation (Placement(transformation(extent={{90,30},{70,50}})));
    equation
      connect(headwater.a1_open, ConstantHeadLevel.b) annotation (Line(points={
              {-69,52},{-62,52},{-62,58},{-73,58}}, color={0,0,255}));
      connect(tailwater.a2_open, fixedTailwaterLevel.b)
        annotation (Line(points={{59,40},{69,40}}, color={0,0,255}));
    end TwoReservoirsWithSource;

    model WaterWayRes
      extends ReservoirBase(conduit(ZL=100, ZR=90));
      HydroPower.SinksAndSources.Fixed_HT ConstantHeadLevel(
        paraOption=false,
        H_const=headwater.H_start[1],
        Hmax=headwater.Hmax[1],
        depth=headwater.depth[1])
        annotation (Placement(transformation(extent={{-96,42},{-76,62}})));
      HydroPower.SinksAndSources.Fixed_HT fixedTailwaterLevel(
        paraOption=false,
        H_const=tailwater.H_start[tailwater.n],
        Hmax=tailwater.Hmax[tailwater.n],
        depth=tailwater.depth[tailwater.n])
        annotation (Placement(transformation(extent={{96,30},{76,50}})));
      HydroPower.HydroSystems.PipeValve pipeValve(
        m_dot_nom=1000*100e6/(1e3*9.81*90),
        dp_nom=101325*90/9.81,
        d_nom(displayUnit="kg/m3"),
        ZL=90) annotation (Placement(transformation(extent={{12,30},{32,50}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(
        D=10,
        deltZ=50,
        Vol=100)
        annotation (Placement(transformation(extent={{-14,30},{6,50}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-0.9,
        duration=1,
        offset=1e-6,
        startTime=2000)
        annotation (Placement(transformation(extent={{-22,68},{-2,88}})));
    equation
      connect(headwater.a1_open, ConstantHeadLevel.b)
        annotation (Line(points={{-69,52},{-75,52}}, color={0,0,255}));
      connect(tailwater.a2_open, fixedTailwaterLevel.b)
        annotation (Line(points={{59,40},{75,40}}, color={0,0,255}));
      connect(pipeValve.a, surgeTank.b)
        annotation (Line(points={{11,40},{7,40}}, color={0,0,255}));
      connect(surgeTank.a, conduit.b)
        annotation (Line(points={{-15,40},{-21,40}}, color={0,0,255}));
      connect(pipeValve.b, tailwater.a1_pipe) annotation (Line(points={{33,40},
              {34,40},{34,28},{37,28}}, color={0,0,255}));
      connect(ramp.y, pipeValve.ValveCtrl)
        annotation (Line(points={{-1,78},{22,78},{22,51}}, color={0,0,127}));
    end WaterWayRes;

    model WaterWayResWithClosingValve
      extends WaterWayRes(ramp(
          height=-1,
          offset=1,
          startTime=300), system_HPL(Q_start=116));
      annotation (experiment(
          StopTime=600,
          Tolerance=1e-05,
          __Dymola_Algorithm="Radau"));
    end WaterWayResWithClosingValve;

    model SundsbarmWaterWay
      extends ReservoirBase(
        conduit(
          endD={5.8,5.8},
          L=6600,
          ZL=564,
          ZR=541.5),
        headwater(
          Hmax=ones(headwater.n)*(564 + 48 + 5),
          depth=ones(headwater.n)*(48 + 5),
          depthIntake={0,0},
          H_start=ones(headwater.n)*(564 + 48),
          steadyState=true),
        tailwater(Hmax=ones(tailwater.n)*(110 + 5 + 3), depth=ones(tailwater.n)
              *(5 + 3)),
        system_HPL(Q_start=24));
      HydroPower.SinksAndSources.Fixed_HT ConstantHeadLevel(
        paraOption=false,
        H_const=564 + 48,
        Hmax=564 + 48 + 5,
        depth=48 + 5)
        annotation (Placement(transformation(extent={{-96,42},{-76,62}})));
      HydroPower.SinksAndSources.Fixed_HT fixedTailwaterLevel(
        paraOption=false,
        H_const=110 + 5,
        Hmax=110 + 5 + 3,
        depth=5 + 3)
        annotation (Placement(transformation(extent={{96,30},{76,50}})));
      HydroPower.HydroSystems.PipeValve PressureShaft(
        endD={3,3},
        m_dot_nom=24e3,
        dp_nom=480*9.81*1e3,
        d_nom(displayUnit="kg/m3"),
        L=724,
        ZL=541.5,
        ZR=112.5)
        annotation (Placement(transformation(extent={{-28,-26},{-8,-6}})));
      HydroPower.HydroSystems.SurgeTank surgeTank(
        D=3.6,
        deltZ=150,
        H2L=0.87,
        Vol=100)
        annotation (Placement(transformation(extent={{-56,-26},{-36,-6}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-0.9,
        duration=10,
        offset=1,
        startTime=100)
        annotation (Placement(transformation(extent={{-22,68},{-2,88}})));
      HydroPower.HydroSystems.Pipe tailRace(
        endD={5.8,5.8},
        ZL=110.5,
        ZR=110,
        horizontalIcon=true,
        L=600) annotation (Placement(transformation(extent={{28,-18},{48,2}})));
      HydroPower.HydroSystems.HydroComponents.Containers.ClosedVolume
        turbinehouse(D=5.8, L=2)
        annotation (Placement(transformation(extent={{2,-22},{22,-2}})));
    equation
      connect(headwater.a1_open, ConstantHeadLevel.b)
        annotation (Line(points={{-69,52},{-75,52}}, color={0,0,255}));
      connect(tailwater.a2_open, fixedTailwaterLevel.b)
        annotation (Line(points={{59,40},{75,40}}, color={0,0,255}));
      connect(PressureShaft.a, surgeTank.b)
        annotation (Line(points={{-29,-16},{-35,-16}}, color={0,0,255}));
      connect(surgeTank.a, conduit.b) annotation (Line(points={{-57,-16},{-58,
              -16},{-58,14},{-10,14},{-10,40},{-21,40}}, color={0,0,255}));
      connect(ramp.y, PressureShaft.ValveCtrl) annotation (Line(points={{-1,78},
              {6,78},{6,8},{-18,8},{-18,-5}}, color={0,0,127}));
      connect(PressureShaft.b, turbinehouse.a) annotation (Line(points={{-7,-16},
              {-2,-16},{-2,-12},{2,-12}}, color={0,0,255}));
      connect(tailRace.a, turbinehouse.b) annotation (Line(points={{27,-8},{24,
              -8},{24,-12},{22,-12}}, color={0,0,255}));
      connect(tailRace.b, tailwater.a1_pipe) annotation (Line(points={{49,-8},{
              60,-8},{60,-10},{62,-10},{62,14},{26,14},{26,28},{37,28}}, color=
              {0,0,255}));
      annotation (Diagram(coordinateSystem(extent={{-140,-100},{140,100}})),
          Icon(coordinateSystem(extent={{-140,-100},{140,100}})));
    end SundsbarmWaterWay;
  end Tut6;
  annotation (uses(Modelica(version="4.0.0"), HydroPower(version="2.17")));
end MyFM;
