"use strict";

requirejs.config({
  baseUrl: 'scripts',
  paths: {
    jquery: '../vendor/jquery/jquery-3.2.0.min',
    underscore: '../vendor/underscore/underscore.min',
    concrete: '../vendor/concrete/concrete.min'
  },
  shim: {
    'jquery': {
      exports: '$'
    },
    'underscore': {
      exports: '_'
    },
    'concrete': {
      exports: 'Concrete'
    }
  }
});


requirejs(['app'], function(app) {

  $(document).ready(function() {
    var application = new app.App();
    application.run();
  });

});
