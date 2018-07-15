const Client = require('node-rest-client-promise').Client({});

//var prom = Client.getPromise("https://httpbin.org/get").then(
//  function(result) {
//    console.log("RESULT: %s ", JSON.stringify(result.response.statusCode))
//  }
//)

var args = {
    path: {
        "id": "abd8c6ee1ea138a532b4bc25976a011c84c942c41203e943fe5f6c6f5e94a341"
    },
    headers: {
        "test-header": "client-api"
    },
    method: 'HEAD',
    mode: 'no-cors'
}

Client.registerMethodPromise("getAllTxs", "http://cardanoexplorer.com/api/txs/last", "GET");
Client.registerMethodPromise("getTxSummary", "http://cardanoexplorer.com/api/txs/summary/${id}", "GET");

Client.methods.getAllTxs()
  .then(
      function (result) {

        processTransactions(result.data.Right)
        //console.log("ARGS: ", result)
      }
  )
  .catch(
    function (e) {
      return e.code === 'ENOTFOUND';
    },
    function () {
      return Promise.resolve();
    }
  )




function processTransactions(txs) {
  if (txs.length === 0) {
    return
  }

  console.log("Txs: ", txs)
  console.log("Txs Length: ", txs.length)

  for (var i = 0; i < txs.length; i++) {

    var txHash = txs[i].cteId;

    var links = []
    args.path.id = txHash
    Client.methods.getTxSummary(args)
      .then(function (result) {
        var tx = result.data.Right
        //console.log("XXXXXXXXXXXXResult: ", result.data.Right)
        processTransaction(tx)
    
      })
      .catch(function (e) {
            console.log("ERRO: ", err)
            return e.code === 'ENOTFOUND';
          },
          function () {
            console.log("NOERRO: " )
            return Promise.resolve();
          }
      )
  } 
}

function processTransaction(tx) {
  
  console.log("################################################")
  console.log("Result: ", tx.ctsId)

  var links = []
  var link = {}

  var inputs = tx.ctsInputs
  for (var i = 0; i < inputs.length; i++) {
    link = {
      from: inputs[i][0],
      to:   tx.ctsId,
      value: inputs[i][1].getCoin,
      t: "i"
    }
    console.log("Link: ", JSON.stringify(link))

    links.push(link)
  }

  var outputs = tx.ctsOutputs
  for (var i = 0; i < outputs.length; i++) {
    link = {
      from:  tx.ctsId,
      to:    outputs[i][0],
      value: outputs[i][1].getCoin,
      t: "o"
    }
    console.log("Link: ", JSON.stringify(link))

    links.push(link)
  }
}
