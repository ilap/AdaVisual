module.exports.main = function () {
  //var graph = require('ngraph.generators').balancedBinTree(6);
  var graph = require('ngraph.graph')();
  var createPixiGraphics = require('ngraph.pixi');

  var createNodes = require('./lib/mock_data.js')(graph)

  var layout = createLayout(graph);


  var pixiGraphics = createPixiGraphics(graph, layout);
  // setup our custom looking nodes and links:
  pixiGraphics.createNodeUI(require('./lib/createNodeUI'))
    .renderNode(require('./lib/renderNode'))
    .createLinkUI(require('./lib/createLinkUI'))
    .renderLink(require('./lib/renderLink'));

  console.log(pixiGraphics.graphGraphics)
  pixiGraphics.graphGraphics.scale.x = 0.15

  pixiGraphics.graphGraphics.scale.y = 0.15

  //layout = pixiGraphics.layout;
  //var node = graph.getNode(tx) 
  //layout.pinNode(node,true);

  // begin animation loop:
  console.log("AAAA:" + JSON.stringify(createNodes))
  createNodes(0)
  pixiGraphics.run();

}

function createLayout(graph) {
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
