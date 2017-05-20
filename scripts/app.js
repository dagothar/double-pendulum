"use strict";

define(['jquery', 'rk4', 'concrete'], function($, rk4, Concrete) {

  const CONFIG = {
    VIEW_ID:      '#view',
    VIEW_WIDTH:   800,
    VIEW_HEIGHT:  600,
    VIEW_SCALE:   100
  };


  function App() {
    this.t = 0.0;

    this.x = [
      3.0,    // theta1
      0.0,    // dtheta1
      3.0,    // theta2
      0.0     // dtheta2
    ];

    this.pendulum = {
      m1:   1.0,
      m2:   1.0,
      l1:   1.0,
      l2:   1.0,
      g:    9.81
    };

    this.solver            = new rk4.RK4();
    this.running           = false;
    this.interval          = undefined;

    this.viewContainer     = undefined;
    this.view              = undefined;
    this.pendulumLayer     = undefined;
    this.traceLayer        = undefined;
  };


  var model = function(t, u, x, p) {
    var tau1 =  u[0];
    var tau2 =  u[1];
    var th1 =   x[0];
    var dth1 =  x[1];
    var th2 =   x[2];
    var dth2 =  x[3];

    var d2th1 = (-p.m2 * p.l2 * dth1 * dth1 * Math.sin(th1-th2) * Math.cos(th1-th2)  + p.g * p.m2 * Math.sin(th2) * Math.cos(th1-th2)
      - p.m2 * p.l2 * dth2 * dth2 * Math.sin(th1-th2) - (p.m1 + p.m2) * p.g * Math.sin(th1)) / (p.l1 * (p.m1 + p.m2) - p.m2 * p.l1 *
      Math.pow(Math.cos(th1-th2), 2));
    var d2th2 = (p.m2 * p.l2 * dth2 * dth2 * Math.sin(th1-th2) * Math.cos(th1-th2) + p.g * Math.sin(th1) * Math.cos(th1-th2) * (p.m1 + p.m2)
      + p.l1 * dth1 * dth1 * Math.sin(th1 - th2) * (p.m1 + p.m2) - p.g * Math.sin(th2) * (p.m1 + p.m2)) / (
      p.l2 * (p.m1 + p.m2) - p.m2 * p.l2 * Math.pow(Math.cos(th1-th2), 2));

    var dx = [
      x[1],
      d2th1,
      x[3],
      d2th2
    ];

    return dx;
  };


  App.prototype.initialize = function() {
    var self = this;

    this.viewContainer = $(CONFIG.VIEW_ID).get(0);
    this.view = new Concrete.Viewport({
      container: this.viewContainer,
      width: CONFIG.VIEW_WIDTH,
      height: CONFIG.VIEW_HEIGHT
    });
    this.pendulumLayer   = new Concrete.Layer();
    this.traceLayer      = new Concrete.Layer();
    this.view            .add(this.traceLayer).add(this.pendulumLayer);

    $('.button-start').click(function() { self.start(); });
    $('.button-stop').click(function() { self.stop(); });

    this.update();
  };


  App.prototype.drawPendulum = function(ctx, x, p) {
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


  App.prototype.step = function() {
    var self = this;

    this.t += 0.01;
    this.x = this.solver.solve(function(t, u, x) { return model(t, u, x, self.pendulum); }, this.t, [0, 0], this.x, 0.01);

    this.update();
  };


  App.prototype.update = function() {
    this.drawPendulum(this.traceLayer.scene.context, this.x, this.pendulum);
  };


  App.prototype.start = function() {
    var self = this;

    if (!this.running) {
      this.running = true;
      clearInterval(this.interval);
      this.interval = setInterval(function() { self.step(); }, 10);
    }
  };


  App.prototype.stop = function() {
    this.running = false;
    clearInterval(this.interval);
  };


  App.prototype.run = function() {
    this.initialize();
  };


  return {
    App: App
  };

});
