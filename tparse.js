var stdin = process.openStdin();

var data = [];

stdin.addListener("data", function(d) {
    var str = d.toString().trim();
    if(str === '')
    {
        process.stdout.write('\033c');
        console.log('\n=====================================================================\n');
        str = data.join('\n');
        data = [];
        str = str.replace(/vtp::Tag<\(Enum\)(\d+)u, +(\w+)>/gm, '$1:$2');
        str = str.replace(/vtp::Tag<\(Enum\)(\d)u, vtp::Vector<([\w:, ]+)> *>/gm, '$1:($2)');
        str = str.replace(/vtp::Vector<([\w:\(, \)]+)>/gm, '$1');
        str = str.replace(/ \)/gm, ')');
        console.log(str);

        console.log('OK');
    }
    else
        data.push(str);
  });

/*rl.question('Input: ', (str) => {
    str = str.replace(/vtp::Tag<\(Enum\)(\d)u, (\l+)>/gm, '$1:$2');

    console.log('Result:', str);
    rl.close();
});*/

