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
  end Tutorial2;
  annotation (uses(Modelica(version="4.0.0")));
end MyFM;
