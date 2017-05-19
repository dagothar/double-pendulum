"use strict";

define(['jquery', 'rk4', 'concrete'], function($, rk4, Concrete) {

  const CONFIG = {
    VIEW_ID:      '#view',
    VIEW_WIDTH:   800,
    VIEW_HEIGHT:  600
  };


  function App() {
    this._x0 = [
      0.0,    // theta1
      0.0,    // dtheta1
      0.0,    // theta2
      0.0     // dtheta2
    ];

    this._pendulum = {
      m1: 1.0,
      m2: 1.0,
      l1: 1.0,
      l2: 1.0,
      g: 9.81
    };

    this._viewContainer     = undefined;
    this._view              = undefined;
    this._pendulumLayer     = undefined;
    this._traceLayer        = undefined;
  };


  var model = function(t, u, x, p) {
    var tau1 = u[0];
    var tau2 = u[1];
    var th1 = x[0];
    var dth1 = x[1];
    var th2 = x[2];
    var dth2 = x[3];

    var A = (p.m1 + p.m2) * p.l1 * p.l1;
    var B = p.m2 * p.l1 * p.l1 * Math.cos(th1 - th2);
    var C = p.m2 * p.l1 * p.l2 * Math.sin(th1-th2) * dth2 * dth2 + (p.m1 + p.m2) * p.g * p.l1 * Math.sin(th1) - tau1;
    var D = p.m2 * p.l1 * p.l2 * Math.cos(th1 - th2);
    var E = p.m2 * p.l2 * p.l2;
    var F = -p.m2 * p.l1 * p.l2 * Math.sin(th1 - th2) * dth1 * dth1 + p.m2 * p.g * p.l2 * Math.sin(th2) - tau2;

    var d2th1 = (C*E - B*F) / (B*D - A*E);
    var d2th2 = (C*D - A*F) / (A*E - B*D);

    var dx = [
      x[1],
      d2th1,
      x[3],
      d2th2
    ];

    return dx;
  };


  App.prototype._initialize = function() {
    // create view
    this._viewContainer = $(CONFIG.VIEW_ID).get(0);
    this._view = new Concrete.Viewport({
      container: this._viewContainer,
      width: CONFIG.VIEW_WIDTH,
      height: CONFIG.VIEW_HEIGHT
    });
    this._pendulumLayer   = new Concrete.Layer();
    this._traceLayer      = new Concrete.Layer();
    this._view.add(this._pendulumLayer).add(this._traceLayer);
  };


  App.prototype._drawPendulum = function() {
    
  };


  App.prototype.run = function() {
    this._initialize();
    var self = this;

    console.log('!');

    var x = [1, 0, 1, 0];
    var solver = new rk4.RK4();

    for (var t = 0.0; t < 0.1; t += 0.01) {
      x = solver.solve(function(t, u, x) { return model(t, u, x, self._pendulum); }, t, [0, 0], x, 0.01);
      console.log(x);
    }
  };


  App.prototype._update = function() {

  };


  return {
    App: App
  };

});
