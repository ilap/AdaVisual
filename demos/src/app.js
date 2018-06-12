module.exports.main = function () {

  if (typeof window === 'undefined') {
    console.log("In Node.js")
  } else {
    console.log("In Browser")
  }
  var graph = require('ngraph.graph')();

  // TODO: Separate the UI from the business logic
  var createNodes = require('./lib/mock_data.js')(graph)
  var layout = createLayout(graph);

//ILAP:  var createPixiGraphics = require('ngraph.pixi');
//ILAP:  var pixiGraphics = createPixiGraphics(graph, layout);
  // setup our custom looking nodes and links:
//ILAP:  pixiGraphics.createNodeUI(require('./lib/createNodeUI'))
//ILAP:    .renderNode(require('./lib/renderNode'))
//ILAP:    .createLinkUI(require('./lib/createLinkUI'))
//ILAP:    .renderLink(require('./lib/renderLink'));

//ILAP:  console.log(pixiGraphics.graphGraphics)
//ILAP:  pixiGraphics.graphGraphics.scale.x = 0.35

//ILAP:  pixiGraphics.graphGraphics.scale.y = 0.35

  //layout = pixiGraphics.layout;
  //var node = graph.getNode(tx) 
  //layout.pinNode(node,true);

  // begin animation loop:
  console.log("AAAA:" + JSON.stringify(createNodes))
  createNodes(0)
//ILAP:  pixiGraphics.run();

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

