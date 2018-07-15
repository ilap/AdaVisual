module.exports = function (graph) {
  var layout = require('ngraph.forcelayout'),
      physics = require('ngraph.physics.simulator');

  return layout(graph, physics({
          springLength: 80,
          springCoeff: 0.0002,
          dragCoeff: 0.0002,
          gravity: -30,
          theta: 0.7
        }));
}
