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
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation
        (Placement(transformation(
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
  annotation (uses(Modelica(version="4.0.0")));
end MyFM;
