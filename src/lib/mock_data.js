/**
 * @file
 * Provides random nodes and links that simulates transactions
 *
 * 
 */

module.exports = function nodes (graph) {
  
  var result = function createNode (id) {
  
    if ( id === undefined) {
      id = 0
    }
  
    var is = getRandomExp(1, 12, 16)
    var os = getRandomExp(2, 24)

    var tx = graph.addNode('t' + id, { type: 't', value: getRandomExp(100, 1000000, 1000 ) })

    console.log("In: %d, Out: %d, Txs: %s", is, os, JSON.stringify(tx))

    var io = {}
    var rn = 0 
    for ( var i = 0; i < is; i++) {
      rn = getRandomExp(10, 1000000, 400)   
      io = addNode(graph, 'i', i + id, rn)  
      // io = graph.addNode('i' + i + id, { type: 'i', value: getRandomExp(10, 1000000, 400) })
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

/**
 * Construct a node based on its type
 */
var addNode = function (graph, type, id, value) {
  nodeId = type + id
  data = { type: type, value: value }
  console.log("Node ID: %s, data: %s", nodeId, JSON.stringify(data))
  return graph.addNode(nodeId, data)    
}

/**
 * Generates a random integer between min and max.
 *
 * If mean is not null then it generates random number from the exponential distribution with the specified mean
 * Otherwise it only generates a random number between min and max value.
 *
 * @param {Integer} min
 *   Minimum value of the random number.
 * @param {Integer} max 
 *   Maximum value of the random number.
 * @param {Integer} mean 
 *   Mean value for the exponention random number.
 * 
 * @return
 *   the generated random number or -1 on any error.
 */
function getRandomExp(min, max, mean) {

  min = Math.ceil(min);
  max = Math.floor(max);
  result = -1

  if (typeof mean === 'undefined') {
     result = Math.floor(Math.random() * (max - min)) + min; 
  } else {
     result = -Math.log(Math.random()) * mean 
     result = result > max ? max : result <= min ? min : result
     result = Math.floor(result)
  }
  
  return result
}
