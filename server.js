var connect = require('connect');
var serveStatic = require('serve-static');

connect().use(serveStatic(__dirname + '/dist/')).listen(8082, function(){
    console.log('Server running on 8082...');
});
