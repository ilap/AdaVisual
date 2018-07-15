module.exports.main = function () {

  var is_node = false

  if (typeof window === 'undefined') {
    console.log("In Node.js")
    is_node = true
  } else {
    console.log("In Browser")
  }
  var graph = require('ngraph.graph')();

  // TODO: Separate the UI from the business logic
  var createNodes = require('./lib/mock_data.js')(graph)
  var layout = createLayout(graph);
  createNodes(0)

  console.log("AAAA:" + JSON.stringify(createNodes))
  
  if (! is_node) {

    var createPixiGraphics = require('ngraph.pixi');
    var pixiGraphics = createPixiGraphics(graph, layout);
      // setup our custom looking nodes and links:
    pixiGraphics.createNodeUI(require('./ui/createNodeUI'))
      .renderNode(require('./ui/renderNode'))
      .createLinkUI(require('./ui/createLinkUI'))
      .renderLink(require('./ui/renderLink'));

    console.log(pixiGraphics.graphGraphics)
    pixiGraphics.graphGraphics.scale.x = 0.35

    pixiGraphics.graphGraphics.scale.y = 0.35

    layout = pixiGraphics.layout;
    var nodes = graph.getNode("t0") 
    layout.pinNode(nodes,true);

    // begin animation loop:
    pixiGraphics.run();
  }
 
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

