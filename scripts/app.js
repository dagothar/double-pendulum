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
    BTN_CLEAR_ID:   '.button-clear',
    BTN_IMAGE_ID:   '.button-download',
    TIME_ID:        '.time',
    DISTANCE_ID:    '.distance',
    POS1_ID:        '.pos1',
    POS2_ID:        '.pos2',
    EKINETIC_ID:    '.ekinetic',
    EPOTENTIAL_ID:  '.epotential',
    ETOTAL_ID:      '.etotal',
    SLIDER_DT_ID:   '.slider-dt',
    DT_ID:          '.dt',
    VIEW_BOB_SIZE:  5,
    VIEW_ARC_SIZE:  15,
    SIM_DT:         0.01,
    M1_ID:          '.mass1',
    M2_ID:          '.mass2',
    L1_ID:          '.length1',
    L2_ID:          '.length2',
    NUM_ID:         '.num',
    SLIDER_B_ID:    '.slider-damping',
    SLIDER_G_ID:    '.slider-gravitation',
    SLIDER_N_ID:    '.slider-noise',
    DAMPING_ID:     '.damping',
    GRAVITATION_ID: '.gravitation',
    NOISE_ID:       '.noise',
    MAX_LENGTH:     2.4,
    TAU_MAG:        10.0,
    NOISE_MAG:      1,
    INDEX_ID:       '.idx',
  };


  var model = function(t, u, x, p) {
    var tau1 =  u[0];
    var tau2 =  u[1];
    var q1 =   x[0];
    var dq1 =  x[1];
    var q2 =   x[2];
    var dq2 =  x[3];
    var m1 = p.m1;
    var m2 = p.m2;
    var l1 = p.l1;
    var l2 = p.l2;
    var g = p.g;
    var b = p.b;
    var sin = Math.sin;
    var cos = Math.cos;

    var sq1 = sin(q1);
    var sq2 = sin(q2);
    var cq1q2 = cos(q1 - q2);
    var cq1q2_2 = cq1q2*cq1q2;
    var sq1q2 = sin(q1 - q2);
    var s2q12q2 = sin(2*q1 - 2*q2);
    var l1_2 = l1 * l1;
    var l2_2 = l2 * l2;
    var dq1_2 = dq1 * dq1;
    var dq2_2 = dq2 * dq2;
    var m1_2 = m1 * m1;
    var m2_2 = m2 * m2;
    var m1_m2 = m1 * m2;
    var l1_l2 = l1 * l2;
    var den = (m1 + m2 - m2*cq1q2_2);

    var d2th1 = (-l1*tau2*cq1q2 + l2*tau1 - (g*(l1_l2*m2*sin(q1 - 2*q2) + l1_l2*(2*m1+m2)*sq1) + dq1_2*l1_2*l2*m2*s2q12q2)/2 - dq2_2*l1*l2_2*m2*sq1q2) / (l1_2*l2*den) - dq1 * b;
    var d2th2 = (l1*(m1+m2)*tau2 - l2*m2*tau1*cq1q2 + dq1_2*l1_2*l2*(m2_2 + m1_m2)*sq1q2 + (g*(l1_l2*(m2_2 + m1_m2)*sin(2*q1 - q2) - l1_l2*(m2_2 + m1_m2)*sq2) + dq2_2*l1*l2_2*m2_2*s2q12q2)/2) / (l1*l2_2*m2*den) - dq2 * b;
    var dx = [
      x[1],
      d2th1,
      x[3],
      d2th2
    ];

    return dx;
  };


  var jacobian = function(x, p) {
    return {
      dxdth1: p.l1 * Math.cos(x[0]),
      dxdth2: p.l2 * Math.cos(x[2]),
      dydth1: -p.l1 * Math.sin(x[0]),
      dydth2: -p.l2 * Math.sin(x[2])
    };
  };


  var getMousePos = function(e, client) {
    var rect = client.getBoundingClientRect();
    return {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top
    };
  };


  var randomColor = function() {
    var h = Math.floor(360*Math.random());
    return 'hsl(' + h + ', 100%, 50%)';
  };


  function App() {
    this.t = 0.0;
    this.dt = CONFIG.SIM_DT;
    this.tscale = 1.0;
    this.distance = [0.0];

    this.x0 = [
      3.14,    // theta1
      0.0,    // dtheta1
      3.141,    // theta2
      0.0     // dtheta2
    ];
    this.startx = this.x0;
    this.n = 1;
    this.index = 0;
    this.x = [];
    this.tau1 = 0.0;
    this.tau2 = 0.0;
    this.noise = 0.1*CONFIG.NOISE_MAG;

    this.position = [undefined];
    this.previousPosition = [undefined];
    this.energy = undefined;

    this.pendulum = {
      m1:   1.0,
      m2:   1.0,
      l1:   1.0,
      l2:   1.0,
      g:    9.81,
      b:    0.0
    };

    this.damping = 0.001;

    this.solver            = new rk4.RK4();
    this.running           = false;
    this.interval          = undefined;

    this.viewContainer     = undefined;
    this.view              = undefined;
    this.pendulumLayer     = undefined;
    this.traceLayer        = undefined;
    this.gridLayer         = undefined;
    this.traceColor        = [randomColor(), randomColor()];

    this.imageIndex = 0;
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
    $(CONFIG.BTN_CLEAR_ID).click(function() { self.clear(); });
    $(CONFIG.BTN_IMAGE_ID).click(function() { self.download(self.imageIndex++); });
    $(CONFIG.SLIDER_DT_ID).val(100).on('input change', function() {
        self.tscale = Math.pow(1.03271, 100-$(this).val());
        self.update();
    });
    $(CONFIG.SLIDER_B_ID).val(0).on('input change', function() {
      self.pendulum.b = 0.01 * $(this).val();
      self.update();
    });
    $(CONFIG.SLIDER_G_ID).val(100).on('input change', function() {
      self.pendulum.g = 0.01 * 9.81 * $(this).val();
      self.update();
    });
    $(CONFIG.SLIDER_N_ID).val(10).on('input change', function() {
      self.noise = 0.01 * CONFIG.NOISE_MAG * $(this).val();
      self.update();
    });
    $(CONFIG.M1_ID).on('input change', function() {
      self.pendulum.m1 = parseFloat($(this).val());
    });
    $(CONFIG.M2_ID).on('input change', function() {
      self.pendulum.m2 = parseFloat($(this).val());
    });
    $(CONFIG.L1_ID).on('input change', function() {
      var l = parseFloat($(this).val());
      var length = self.pendulum.l2 + l;
      if (length <= CONFIG.MAX_LENGTH) {
        self.pendulum.l1 = l;
        self.update();
      } else {
        $(this).val(self.pendulum.l1);
      }
    });
    $(CONFIG.L2_ID).on('input change', function() {
      var l = parseFloat($(this).val());
      var length = self.pendulum.l1 + l;
      if (length <= CONFIG.MAX_LENGTH) {
        self.pendulum.l2 = l;
        self.update();
      } else {
        $(this).val(self.pendulum.l2);
      }
    });
    $(CONFIG.NUM_ID).val(this.n).on('input change', function() {
      var n = parseInt($(this).val());
      if (n > 0 && n <= 100) {
        for (var i = self.n; i < n; ++i) {
          self.placePendulum(i, self.x[0], self.noise);
        }
        self.n = n;
        self.update();
      }
    });
    $(CONFIG.INDEX_ID).val(1).on('input change', function() {
      var idx = parseInt($(this).val())-1;
      if (idx >= 0 && idx < self.n) {
        self.index = idx;
        self.update();
      } else {
        $(this).val(self.index+1);
      }
    });

    $(CONFIG.VIEW_ID).mousedown(function(e) { self.pullPendulum(e); });

    $(window).keydown(function(e) {
      switch (e.key) {
        case 'q':
          self.tau1 = -CONFIG.TAU_MAG;
          break;
        case 'w':
          self.tau1 = CONFIG.TAU_MAG;
          break;
        case 'o':
          self.tau2 = -CONFIG.TAU_MAG;
          break;
        case 'p':
          self.tau2 = CONFIG.TAU_MAG;
          break;
        default:
          break;
      }
    });

    $(window).keyup(function(e) {
      switch (e.key) {
        case 'q':
        case 'w':
          self.tau1 = 0.0;
          break;
        case 'o':
        case 'p':
          self.tau2 = 0.0;
          break;
        default:
          break;
      }
    });

    $(CONFIG.DIV_START_ID).show();
    $(CONFIG.DIV_STOP_ID).hide();

    for (var i = 0; i < this.n; ++i)
      this.placePendulum(i, this.x0, this.noise);

    for (var i = 0, n = this.n; i < n; ++i)
      this.position[i] = this.calculatePosition(this.x[i], this.pendulum);
    this.energy = this.calculateEnergy(this.x[0], this.pendulum);
    this.update();

    this.drawGrid(this.gridLayer.scene.context);
    this.clear();
  };


  App.prototype.update = function() {
    var idx = this.index;

    for (var i = 0, n = this.n; i < n; ++i) {
      this.previousPosition[i] = this.position[i];
      this.position[i] = this.calculatePosition(this.x[i], this.pendulum);
      var dpos = Math.sqrt(Math.pow(this.position[i].x2 - this.previousPosition[i].x2, 2) + Math.pow(this.position[i].y2 - this.previousPosition[i].y2, 2));
      this.distance[i] += dpos;
    }

    this.pendulumLayer.scene.context.clearRect(0, 0, CONFIG.VIEW_WIDTH, CONFIG.VIEW_HEIGHT);
    this.traceLayer.scene.context.fillStyle = 'rgba(255, 255, 255, 0.01)';
    this.traceLayer.scene.context.fillRect(0, 0, CONFIG.VIEW_WIDTH, CONFIG.VIEW_HEIGHT);

    for (var i = this.n-1; i >= 0; --i) {
      this.drawPendulum(this.pendulumLayer.scene.context, this.position[i], 1.0*i/this.n);
      this.drawTrace(this.traceLayer.scene.context, this.previousPosition[i], this.position[i], this.traceColor[i]);
    }

    $(CONFIG.TIME_ID).text(this.t.toFixed(2) + ' s');
    $(CONFIG.DISTANCE_ID).text(this.distance[idx].toFixed(2) + ' m');
    $(CONFIG.POS1_ID).text(
      '(' + this.position[idx].x1.toFixed(2) + ', '
      + this.position[idx].y1.toFixed(2) + ') m'
    );
    $(CONFIG.POS2_ID).text(
      '(' + this.position[idx].x2.toFixed(2) + ', '
      + this.position[idx].y2.toFixed(2) + ') m'
    );
    $(CONFIG.EKINETIC_ID).text(this.energy.Ek.toFixed(2) + ' J');
    $(CONFIG.EPOTENTIAL_ID).text(this.energy.Ep.toFixed(2) + ' J');
    $(CONFIG.ETOTAL_ID).text(this.energy.E.toFixed(2) + ' J');
    $(CONFIG.DT_ID).text((1/this.tscale).toFixed(2) + 'x');
    $(CONFIG.DAMPING_ID).text(this.pendulum.b.toFixed(2));
    $(CONFIG.GRAVITATION_ID).text((this.pendulum.g/9.81).toFixed(2) + 'x');
    $(CONFIG.NOISE_ID).text((this.noise).toFixed(2));
  };


  App.prototype.invKin = function(state, desiredPos, p, alpha, epsilon, maxIter) {
    var alpha = alpha || 0.1;
    var epsilon = epsilon || 1e-3;
    var maxIter = maxIter || 1000;
    var x = state.slice();

    var error = 0.0;
    var iter = 0;
    do {
      var position = this.calculatePosition(x, p);
      var dx = desiredPos.x - position.x2;
      var dy = desiredPos.y - position.y2;
      error = Math.sqrt(dx*dx + dy*dy);

      var J = jacobian(x, p);
      var dth1 = J.dxdth1 * dx + J.dydth1 * dy;
      var dth2 = J.dxdth2 * dx + J.dydth2 * dy;

      x[0] = x[0] + alpha * dth1;
      x[2] = x[2] + alpha * dth2;

      ++iter;
    } while (error > epsilon && iter < maxIter);

    x[1] = 0.0;
    x[3] = 0.0;

    return x;
  };


  App.prototype.pullPendulum = function(e) {
    var pos = getMousePos(e, $(CONFIG.VIEW_ID).get(0));
    pos.x = (pos.x - CONFIG.VIEW_WIDTH/2)  / CONFIG.VIEW_SCALE;
    pos.y = (pos.y - CONFIG.VIEW_HEIGHT/2)  / CONFIG.VIEW_SCALE;

    var startx = [1, 0, 1.1, 0];
    if (pos.x < 0)
      startx = [-1, 0, -1.1, 0];

    this .t = 0.0;
    this.x[0] = this.invKin(startx, pos, this.pendulum);

    for (var i = 0; i < this.n; ++i)
      this.placePendulum(i, this.x[0], this.noise);

    this.update();
  };


  App.prototype.placePendulum = function(idx, x, noise) {
    if (idx == 0) {
      this.x[0] = x.slice();
    } else {
      this.x[idx] = x.slice();
      this.x[idx][0] += 2*noise*Math.random()-noise;
      this.x[idx][2] += 2*noise*Math.random()-noise;
    }

    this.position[idx] = this.calculatePosition(this.x[idx], this.pendulum);
    this.previousPosition[idx] = this.position[idx];
    this.energy = this.calculateEnergy(this.x[idx], this.pendulum);
    this.traceColor[idx] = randomColor();
    this.distance[idx] = 0.0;
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
    var q1 = x[0];
    var dq1 = x[1];
    var q2 = x[2];
    var dq2 = x[3];
    var m1 = p.m1;
    var m2 = p.m2;
    var l1 = p.l1;
    var l2 = p.l2;
    var g = p.g;
    var sin = Math.sin;
    var cos = Math.cos;

    var dq1_2 = dq1 * dq1;
    var dq2_2 = dq2 * dq2;
    var l1_2 = l1 * l1;
    var l2_2 = l2 * l2;
    var cq1 = cos(q1);
    var cq2 = cos(q2);

    var Ek = (dq1_2*l1_2*(m1+m2) + dq2_2*l2_2*m2)/2 + dq1*dq2*l1*l2*m2*cos(q1 - q2);
    var Epmin = - g*(m2*(l1 + l2) + l1*m1);
    var Ep = - g * (m2*(l1*cq1 + l2*cq2) + l1*m1*cq1) - Epmin;
    var E = Ek + Ep;

    return {
      Ek: Ek,
      Ep: Ep,
      E: E
    };
  };


  App.prototype.step = function() {
    var self = this;

    var tau1 = this.tau1 * (this.pendulum.l1 + this.pendulum.l2);
    var tau2 = this.tau2 * (this.pendulum.l2);

    var dt = this.dt / this.tscale;
    this.t += dt;

    for (var i = 0; i < this.n; ++i) {
      this.x[i] = this.solver.solve(function(t, u, x) { return model(t, u, x, self.pendulum); }, this.t, [tau1, tau2], this.x[i], dt);
    }

    this.energy = this.calculateEnergy(this.x[this.index], this.pendulum);

    this.update();
  };


  App.prototype.drawGrid = function(ctx) {
    ctx.save();
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 1;
    ctx.translate(CONFIG.VIEW_WIDTH/2, CONFIG.VIEW_HEIGHT/2);
    for (var x = -3.5; x <= 3.21; x += 0.5) {
      ctx.moveTo(x*CONFIG.VIEW_SCALE, -2.4*CONFIG.VIEW_SCALE);
      ctx.lineTo(x*CONFIG.VIEW_SCALE, 2.4*CONFIG.VIEW_SCALE);
      ctx.stroke();
      //ctx.lineWidth == 1 ? ctx.lineWidth = 2.5 : ctx.lineWidth = 1;
    }
    for (var y = -2.5; y <= 2.41; y += 0.5) {
      ctx.moveTo(-3.2*CONFIG.VIEW_SCALE, y*CONFIG.VIEW_SCALE);
      ctx.lineTo(3.2*CONFIG.VIEW_SCALE, y*CONFIG.VIEW_SCALE);
      ctx.stroke();
      //ctx.lineWidth == 1 ? ctx.lineWidth = 5 : ctx.lineWidth = 1;
    }
    ctx.fill();
    ctx.restore();
  };


  App.prototype.drawPendulum = function(ctx, position, transparency) {
    var x1 = position.x1;
    var y1 = position.y1;
    var x2 = position.x2;
    var y2 = position.y2;

    var c = (255*transparency).toFixed(0);
    var color = 'rgb(' + c + ', ' + c + ', ' + c + ')';

    ctx.save();
    ctx.fillStyle = color;
    ctx.strokeStyle = color;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.lineWidth = 3;

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

    ctx.strokeStyle = 'red';
    if (this.tau1 > 0) {
      ctx.beginPath();
      ctx.arc(0, 0, CONFIG.VIEW_ARC_SIZE, -this.x[0][0]+Math.PI/2-1*this.tau1/CONFIG.TAU_MAG, -this.x[0][0]+Math.PI/2);
      ctx.stroke();
    } else if (this.tau1 < 0) {
      ctx.beginPath();
      ctx.arc(0, 0, CONFIG.VIEW_ARC_SIZE, -this.x[0][0]+Math.PI/2, -this.x[0][0]+Math.PI/2-1*this.tau1/CONFIG.TAU_MAG);
      ctx.stroke();
    }
    if (this.tau2 > 0) {
      ctx.beginPath();
      ctx.arc(x1*CONFIG.VIEW_SCALE, y1*CONFIG.VIEW_SCALE, CONFIG.VIEW_ARC_SIZE, -this.x[0][2]+Math.PI/2-1*this.tau2/CONFIG.TAU_MAG, -this.x[0][2]+Math.PI/2);
      ctx.stroke();
    } else if (this.tau2 < 0) {
      ctx.beginPath();
      ctx.arc(x1*CONFIG.VIEW_SCALE, y1*CONFIG.VIEW_SCALE, CONFIG.VIEW_ARC_SIZE, -this.x[0][2]+Math.PI/2, -this.x[0][2]+Math.PI/2-1*this.tau2/CONFIG.TAU_MAG);
      ctx.stroke();
    }
    ctx.restore();
  };


  App.prototype.drawTrace = function(ctx, prevPos, pos, color) {
    if (!prevPos || !pos) return;

    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineWidth = 2;

    ctx.translate(CONFIG.VIEW_WIDTH/2, CONFIG.VIEW_HEIGHT/2);
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(prevPos.x2*CONFIG.VIEW_SCALE, prevPos.y2*CONFIG.VIEW_SCALE);
    ctx.lineTo(pos.x2*CONFIG.VIEW_SCALE, pos.y2*CONFIG.VIEW_SCALE);
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  };


  App.prototype.clear = function() {
    var ctx = this.traceLayer.scene.context;
    ctx.clearRect(0, 0, CONFIG.VIEW_WIDTH, CONFIG.VIEW_HEIGHT);
    ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
    ctx.fillRect(0, 0, CONFIG.VIEW_WIDTH, CONFIG.VIEW_HEIGHT);
  };


  App.prototype.reset = function() {
    this.stop();
    this.clear();

    this.n = 1;
    this.t = 0.0;
    this.distance = [0.0];
    this.x = [this.x0];
    for (var i = 0; i < this.n; ++i) {
      this.position[i] = this.calculatePosition(this.x[i], this.pendulum);
      this.previousPosition[i] = this.position[i];
    }
    this.energy = this.calculateEnergy(this.x[0], this.pendulum);

    $(CONFIG.SLIDER_DT_ID).val(100);
    $(CONFIG.SLIDER_B_ID).val(0);
    $(CONFIG.SLIDER_G_ID).val(100);
    $(CONFIG.SLIDER_N_ID).val(10);
    $(CONFIG.M1_ID).val(1.0);
    $(CONFIG.M2_ID).val(1.0);
    $(CONFIG.L1_ID).val(1.0);
    $(CONFIG.L2_ID).val(1.0);
    $(CONFIG.NUM_ID).val(1);
    $(CONFIG.INDEX_ID).val(1);

    this.pendulum = {
      m1:   1.0,
      m2:   1.0,
      l1:   1.0,
      l2:   1.0,
      g:    9.81,
      b:    0.0
    };

    this.update();
  };


  App.prototype.download = function(i) {
    this.view.toScene().download({ fileName: 'image' + i + '.png' });
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
