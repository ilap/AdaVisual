class CryptoCurrency {
  constructor(name, parser) {
    this.name = name
    this.parser = parser
  }

  startParser() {
    this.parser.start()
  }

  stopParser() {
    this.parser.stop()
  }
}

function graph () {
  var addNode = function (node) {
    console.log("Added node: ", node)
  }
  return addNode
}

class Parser {
  constructor(graph) {
    this.graph = graph
  }

  start() {
    this.parseTx("NeTX")
  }

  parseTxs() {
  }

  parseTx(tx) {
    this.graph()("AAAAAA")
  } 
}


class Ada extends CryptoCurrency {

}


var parser = new Parser(graph)
var ada = new Ada("ada", parser) 

ada.startParser()

