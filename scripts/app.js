"use strict";

define(['jquery', 'rk4', 'concrete'], function($, rk4, Concrete) {

  const CONFIG = {
    VIEW_ID:        '#view',
    VIEW_WIDTH:     640,
    VIEW_HEIGHT:    480,
    VIEW_SCALE:     100,
    BTN_RESET_ID:   '.button-reset',
    BTN_START_ID:   '.button-start',
    DIV_START_ID:   '.button-start-div',
    BTN_STOP_ID:    '.button-stop',
    DIV_STOP_ID:    '.button-stop-div',
    TIME_ID:        '.time',
    DISTANCE_ID:    '.distance',
    POS1_ID:        '.pos1',
    POS2_ID:        '.pos2',
    EKINETIC_ID:    '.ekinetic',
    EPOTENTIAL_ID:  '.epotential',
    ETOTAL_ID:      '.etotal',
    VIEW_BOB_SIZE:  5
  };


  function App() {
    this.t = 0.0;
    this.distance = 0.0;

    this.x0 = [
      3.14,    // theta1
      0.0,    // dtheta1
      3.14,    // theta2
      0.0     // dtheta2
    ];
    this.x = this.x0;

    this.position = undefined;
    this.previousPosition = undefined;
    this.energy = undefined;

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
    this.gridLayer         = undefined;
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
    this.gridLayer       = new Concrete.Layer();
    this.view            .add(this.gridLayer).add(this.traceLayer).add(this.pendulumLayer);

    $(CONFIG.BTN_RESET_ID).click(function() { self.reset(); });
    $(CONFIG.BTN_START_ID).click(function() { self.start(); });
    $(CONFIG.BTN_STOP_ID).click(function() { self.stop(); });

    $(CONFIG.DIV_START_ID).show();
    $(CONFIG.DIV_STOP_ID).hide();

    this.position = this.calculatePosition(this.x, this.pendulum);
    this.energy = this.calculateEnergy(this.x, this.pendulum);
    this.update();
  };


  App.prototype.drawPendulum = function(ctx, position) {
    var x1 = position.x1;
    var y1 = position.y1;
    var x2 = position.x2;
    var y2 = position.y2;

    ctx.save();
    ctx.fillStyle = 'black';
    ctx.strokeStyle = 'black';
    ctx.lineCap = 'round';
    ctx.lineWidth = 3;

    ctx.clearRect(0, 0, 800, 600);
    ctx.translate(CONFIG.VIEW_WIDTH/2, CONFIG.VIEW_HEIGHT/2);
    ctx.beginPath();
    ctx.arc(x1*CONFIG.VIEW_SCALE, y1*CONFIG.VIEW_SCALE, CONFIG.VIEW_BOB_SIZE, 0, 2*Math.PI);
    ctx.closePath();
    ctx.fill();
    ctx.beginPath();
    ctx.arc(x2*CONFIG.VIEW_SCALE, y2*CONFIG.VIEW_SCALE, CONFIG.VIEW_BOB_SIZE, 0, 2*Math.PI);
    ctx.closePath();
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(x1*CONFIG.VIEW_SCALE, y1*CONFIG.VIEW_SCALE);
    ctx.lineTo(x2*CONFIG.VIEW_SCALE,y2*CONFIG.VIEW_SCALE);
    ctx.stroke();
    ctx.restore();
  };


  App.prototype.drawTrace = function(ctx, prevPos, pos) {
    if (!prevPos || !pos) return;

    ctx.save();
    ctx.lineCap = 'round';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.025)';
    ctx.lineWidth = 2;

    ctx.fillRect(0, 0, CONFIG.VIEW_WIDTH, CONFIG.VIEW_HEIGHT);
    ctx.translate(CONFIG.VIEW_WIDTH/2, CONFIG.VIEW_HEIGHT/2);

    /*ctx.strokeStyle = 'blue';
    ctx.beginPath();
    ctx.moveTo(prevPos.x1*CONFIG.VIEW_SCALE, prevPos.y1*CONFIG.VIEW_SCALE);
    ctx.lineTo(pos.x1*CONFIG.VIEW_SCALE, pos.y1*CONFIG.VIEW_SCALE);
    ctx.closePath();
    ctx.stroke();*/

    ctx.strokeStyle = 'red';
    ctx.beginPath();
    ctx.moveTo(prevPos.x2*CONFIG.VIEW_SCALE, prevPos.y2*CONFIG.VIEW_SCALE);
    ctx.lineTo(pos.x2*CONFIG.VIEW_SCALE, pos.y2*CONFIG.VIEW_SCALE);
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  };


  App.prototype.calculatePosition = function(x, p) {
    var x1 = p.l1 * Math.sin(x[0]);
    var y1 = p.l1 * Math.cos(x[0]);
    var x2 = x1 + p.l2 * Math.sin(x[2]);
    var y2 = y1 + p.l2 * Math.cos(x[2]);

    return {
      x1: x1,
      y1: y1,
      x2: x2,
      y2: y2
    };
  };


  App.prototype.calculateEnergy = function(x, p) {
    var Ek = 0.5 * (p.m1 + p.m2) * p.l1 * p.l1 * x[1] * x[1] + 0.5 * p.m2 * p.l2 * x[3] * x[3]
      + p.m2 * p.l1 * p.l2 * x[1] * x[3] * Math.cos(x[0] - x[2]);
    var Ep = - (p.m1 + p.m2) * p.g * p.l1 * Math.cos(x[0]) - p.m2 * p.g * p.l2 * Math.cos(x[2]);
    var E = Ek + Ep;

    return {
      Ek: Ek,
      Ep: Ep,
      E: E
    };
  };


  App.prototype.step = function() {
    var self = this;

    this.t += 0.01;
    this.x = this.solver.solve(function(t, u, x) { return model(t, u, x, self.pendulum); }, this.t, [0, 0], this.x, 0.01);
    this.previousPosition = this.position;
    this.position = this.calculatePosition(this.x, this.pendulum);
    this.energy = this.calculateEnergy(this.x, this.pendulum);
    var dpos = Math.sqrt(Math.pow(this.position.x2 - this.previousPosition.x2, 2) + Math.pow(this.position.y2 - this.previousPosition.y2, 2));
    this.distance += dpos;

    this.update();
  };


  App.prototype.update = function() {
    this.drawPendulum(this.pendulumLayer.scene.context, this.position);
    this.drawTrace(this.traceLayer.scene.context, this.previousPosition, this.position);

    $(CONFIG.TIME_ID).text(this.t.toFixed(2) + ' s');
    $(CONFIG.DISTANCE_ID).text(this.distance.toFixed(2) + ' m');
    $(CONFIG.POS1_ID).text(
      '(' + this.position.x1.toFixed(2) + ', '
      + this.position.y1.toFixed(2) + ') m'
    );
    $(CONFIG.POS2_ID).text(
      '(' + this.position.x2.toFixed(2) + ', '
      + this.position.y2.toFixed(2) + ') m'
    );
    $(CONFIG.EKINETIC_ID).text(this.energy.Ek.toFixed(2) + ' J');
    $(CONFIG.EPOTENTIAL_ID).text(this.energy.Ep.toFixed(2) + ' J');
    $(CONFIG.ETOTAL_ID).text(this.energy.E.toFixed(2) + ' J');
  };


  App.prototype.clear = function() {
    var ctx = this.traceLayer.scene.context;
    ctx.clearRect(0, 0, CONFIG.VIEW_WIDTH, CONFIG.VIEW_HEIGHT);
  };


  App.prototype.reset = function() {
    this.stop();
    this.clear();
    this.t = 0.0;
    this.distance = 0.0;
    this.x = this.x0;
    this.position = this.calculatePosition(this.x, this.pendulum);
    this.previousPosition = this.position;
    this.energy = this.calculateEnergy(this.x, this.pendulum);
    this.update();
  };


  App.prototype.start = function() {
    var self = this;

    if (!this.running) {
      this.running = true;
      clearInterval(this.interval);
      this.interval = setInterval(function() { self.step(); }, 10);
    }

    $(CONFIG.DIV_START_ID).hide();
    $(CONFIG.DIV_STOP_ID).show();
  };


  App.prototype.stop = function() {
    this.running = false;
    clearInterval(this.interval);

    $(CONFIG.DIV_START_ID).show();
    $(CONFIG.DIV_STOP_ID).hide();
  };


  App.prototype.run = function() {
    this.initialize();
  };


  return {
    App: App
  };

});
