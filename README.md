# Cardano Blockchain Visual

# Websocket, Socket.IO and Rest-API descriptions

## BTC

It uses [Websocket API](https://blockchain.info/api/api_websocket) of blockchain.info by subscribing to the "Unconfirmed transactions" ({"op":"unconfirmed_sub"}).

When a message is received it parses the JSON.

Gets the hash of utx (txHash)
Creates links for each inputs/outputs: 
input.prev_out_addr --> txHash
txHash --> output.addr

``` json
{ from: i1, to: txHash, t: 'i', value: 0.01 }
...
{ from: iN, to: txHash, t: 'i', value: 0.13 }


{ from: txHash, to: o1, t: 'o', value: 0.021 }
...
{ from: txHash, to: oN, t: 'o', value: 0.69 }
```
Then for each link it creates a node and link that node to the transaction.

## ADA

The socket.io does not work as Haskell's socket.io is not compatible /w the Node.js one.
It connects to the API in every 20 secs and check's whether the txs are updated, means check tha saved last tx is not  same with the received last one.

Then it processes every txs.
Gets the hash of tx (ctsId)
Creates links for each inputs/outputs: 
ctsInputs[i][0] -> ctsId
ctsId --> ctsOutputs[i][0]

``` json
{ from: i1, to: ctsId, t: 'i', value: 0.01 }
...
{ from: iN, to: ctsId, t: 'i', value: 0.13 }


{ from: ctsId, to: o1, t: 'o', value: 0.021 }
...
{ from: ctsId, to: oN, t: 'o', value: 0.69 }
```
Then for each link it creates a node and link that node to the transaction.

# Server Install & Run

 $ npm install connect serve-static node-rest-client

 $ node server.js

Server running on 8081...

Open browser 

# Blockchain Visual
bash -x ./build.sh
+ npm install
+ npm run build



# Credits
