module.exports = function nodes (graph) {
  
  var result = function createNode (id) {
  
    if ( id === undefined) {
      id = 0
    }
  
    var is = getRandomExp(1, 12, 16)
    var os = getRandomExp(2, 24, 16)
     
    var tx = graph.addNode('t' + id, { type: 't', value: getRandomExp(100, 1000000, 1000 ) })
    console.log("In: %d, Out: %d, Txs: %d", is, os, tx)

    var io = {}
    for ( var i = 0; i < is; i++) {
      io = graph.addNode('i' + i + id, { type: 'i', value: getRandomExp(10, 1000000, 400) })
      graph.addLink(io.id, tx.id)
    }
    for ( var i = 0; i < os; i++) {
      io = graph.addNode('o' + i + id, { type: 'o', value: getRandomExp(10, 1000000, 400) })
      graph.addLink(tx.id, io.id)
    }
  
    if (id < 100) {
      setTimeout(function () {
        id++
        createNode(id.toString() )
      }, getRandomExp(500, 5000, 1000))
    }
  }
  
  return result
}

function getRandomInt(min, max) {
  min = Math.ceil(min);
  max = Math.floor(max);
  return Math.floor(Math.random() * (max - min)) + min;
}

function getRandomExp(min, max, mean) {
  min = Math.ceil(min);
  max = Math.floor(max);

  var result = -Math.log(Math.random()) * mean 
  result = result > max ? max : result <= min ? min : result
  
  return Math.floor(result);
}
