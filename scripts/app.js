"use strict";

define(['jquery', 'rk4', 'concrete'], function($, rk4, Concrete) {

  const CONFIG = {
    VIEW_ID:      '#view',
    VIEW_WIDTH:   800,
    VIEW_HEIGHT:  600,
    VIEW_SCALE:   100
  };


  function App() {
    this._t = 0.0;

    this._x = [
      2.0,    // theta1
      0.0,    // dtheta1
      2.0,    // theta2
      0.0     // dtheta2
    ];

    this._pendulum = {
      m1:   1.0,
      m2:   1.0,
      l1:   1.0,
      l2:   1.0,
      g:    9.81
    };

    this._solver            = new rk4.RK4();
    this._running           = false;
    this._interval          = undefined;

    this._viewContainer     = undefined;
    this._view              = undefined;
    this._pendulumLayer     = undefined;
    this._traceLayer        = undefined;
  };


  var model = function(t, u, x, p) {
    var tau1 =  u[0];
    var tau2 =  u[1];
    var th1 =   x[0];
    var dth1 =  x[1];
    var th2 =   x[2];
    var dth2 =  x[3];

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
    var self = this;

    // create view
    this._viewContainer = $(CONFIG.VIEW_ID).get(0);
    this._view = new Concrete.Viewport({
      container: this._viewContainer,
      width: CONFIG.VIEW_WIDTH,
      height: CONFIG.VIEW_HEIGHT
    });
    this._pendulumLayer   = new Concrete.Layer();
    this._traceLayer      = new Concrete.Layer();
    this._view            .add(this._pendulumLayer).add(this._traceLayer);

    $('.button-start').click(function() { self._start(); });
    $('.button-stop').click(function() { self._stop(); });
  };


  App.prototype._drawPendulum = function(ctx, x, p) {
    var x1 = p.l1 * Math.sin(x[0]);
    var y1 = p.l1 * Math.cos(x[0]);
    var x2 = x1 + p.l2 * Math.sin(x[2]);
    var y2 = y1 + p.l2 * Math.cos(x[2]);

    ctx.fillStyle = 'black';
    ctx.strokeStyle = 'black';
    ctx.lineWidth = 3;

    ctx.save();
    ctx.clearRect(0, 0, 800, 600);
    ctx.beginPath();
    ctx.arc(CONFIG.VIEW_WIDTH/2, CONFIG.VIEW_HEIGHT/2, 5, 0, 2*Math.PI);
    ctx.closePath();
    ctx.fill();
    ctx.beginPath();
    ctx.arc(CONFIG.VIEW_WIDTH/2+x1*CONFIG.VIEW_SCALE, CONFIG.VIEW_HEIGHT/2+y1*CONFIG.VIEW_SCALE, 5, 0, 2*Math.PI);
    ctx.closePath();
    ctx.fill();
    ctx.beginPath();
    ctx.arc(CONFIG.VIEW_WIDTH/2+x2*CONFIG.VIEW_SCALE, CONFIG.VIEW_HEIGHT/2+y2*CONFIG.VIEW_SCALE, 5, 0, 2*Math.PI);
    ctx.closePath();
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(CONFIG.VIEW_WIDTH/2, CONFIG.VIEW_HEIGHT/2);
    ctx.lineTo(CONFIG.VIEW_WIDTH/2+x1*CONFIG.VIEW_SCALE, CONFIG.VIEW_HEIGHT/2+y1*CONFIG.VIEW_SCALE);
    ctx.lineTo(CONFIG.VIEW_WIDTH/2+x2*CONFIG.VIEW_SCALE, CONFIG.VIEW_HEIGHT/2+y2*CONFIG.VIEW_SCALE);
    ctx.stroke();
    ctx.restore();
  };


  App.prototype._step = function() {
    var self = this;

    this._t += 0.01;
    this._x = this._solver.solve(function(t, u, x) { return model(t, u, x, self._pendulum); }, this._t, [0, 0], this._x, 0.01);

    this._drawPendulum(this._traceLayer.scene.context, this._x, this._pendulum);
  };


  App.prototype._start = function() {
    var self = this;

    if (!this._running) {
      this._running = true;
      clearInterval(this._interval);
      this._interval = setInterval(function() { self._step(); }, 10);
    }
  };


  App.prototype._stop = function() {
    this._running = false;
    clearInterval(this._interval);
    console.log('!');
  };


  App.prototype.run = function() {
    this._initialize();
  };


  App.prototype._update = function() {

  };


  return {
    App: App
  };

});
