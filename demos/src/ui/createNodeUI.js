module.exports = function (node) {
  console.log("ANM: " + JSON.stringify(node))
  return new AnimatedNode(node);
}

var colorLookup = [0x00FFFF, 0xFF0000, 0x00FF00];

function AnimatedNode(node) {
  var color = node.data.type === 't' ? 0 : node.data.type === 'o' ? 1 : 2

  this.color = colorLookup[color]
  this.frame = Math.random();
  this.width = Math.random() * 5 + 5;
  this.v = 1 - Math.random() * 0.01;
}

AnimatedNode.prototype.renderFrame = function() {
  if (this.frame < 0.6) {
    this.frame = 1;
    //this.color = colorLookup[(Math.random() * colorLookup.length)|0];
    this.width = Math.random() * 5 + 5;
    this.v = 0.99999 - Math.random() * 0.01;
  }

  this.frame *= this.v;
  this.width *= this.v;
}
