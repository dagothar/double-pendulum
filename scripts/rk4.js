define([], function() {

  function RK4() {

    //! Solves a step of ODE
    this.solve = function(fun, t, u, x, dt) {
      // calculate derivatives k1
      var k1 = fun(t, u, x);

      // update state
      var x1 = x.slice();
      for (var i = 0; i < x.length; ++i) {
        x1[i] += k1[i] * dt/2;
      }

      // calculate derivatives k2
      var k2 = fun(t+dt/2, u, x1);

      // update state
      var x2 = x.slice();
      for (var i = 0; i < x.length; ++i) {
        x2[i] += k2[i] * dt/2;
      }

      // calculate derivatives k2
      var k3 = fun(t+dt/2, u, x2);

      // update state
      var x3 = x.slice();
      for (var i = 0; i < x.length; ++i) {
        x3[i] += k3[i] * dt;
      }

      // calculate derivatives k4
      var k4 = fun(t+dt, u, x3);

      // update state
      var nx = x.slice();
      for (var i = 0; i < x.length; ++i) {
        nx[i] += dt/6 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
      }

      return nx;
    };

  };


  return {
    RK4: RK4
  };

});
