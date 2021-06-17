var arrayToTable = function (data, attrs) {
    var table = $('<table />'),
        thead,
        tbody,
        tfoot,
        rows = [],
        row,
        link,
        i,
        j;

    table.attr(attrs);

    table.addClass("ui celled table");

    // loop through all the rows, we will deal with tfoot and thead later
    for (i = 0; i < data.length; i = i + 1) {
        row = $('<tr />');
        link = data[i].pop();
        //row.on("click", load_link(link));

        // Make all rows except header row clickable
        if (i != 0) {

            // Use custom URL from row if one is present (but do not show the link in the table)
            var foundLink = false;
            for (j = 0; j < data[i].length; j = j + 1) {
                if (data[i][j].search(/http/i) != -1) {  // custom URL
                    // Get URL and text before and after it
                    var result = data[i][j].match(/(.*)(http[^\s\(\)]*)(.*)/i);
                    link = result[2];
                    console.log("cell before: " + data[i][j]);

                    // Remove URL from cell
                    data[i][j] = result[1] + result[3];
                    console.log("link: " + link);
                    console.log("cell after: " + data[i][j]);
                    foundLink = true;
                    break;
                }
                else if ((j == 0) && (data[i][j].search(/\//i) != -1)) {  // TIREX lab within sub-folder
                    // Get TIREX folder and text before and after it
                    var result = data[i][j].match(/(.*)\((.*)\)(.*)/i);
                    link = result[2];
                    console.log("cell before: " + data[i][j]);

                    // Remove folder name from cell
                    data[i][j] = result[1] + result[3];
                    console.log("link: " + link);
                    console.log("cell after: " + data[i][j]);
                    break;
                }
            }

            // Open URL in new window if URL was present
            if (foundLink == true) {
                row.attr('onclick', 'window.open("'+link+'");');
            }
            // Otherwise, open TIREX page
            else {
                row.attr('onclick', 'load_link("'+link+'");');
            }
        }

        for (j = 0; j < data[i].length; j = j + 1) {
            if (i === 0) {
                row.append($('<th style="text-align:center" />').html(data[i][j]));
            } else {
                if (j === 0) {
                    row.append($('<td />').html(data[i][j]));
                } else {
                    row.append($('<td style="text-align:center" />').html(data[i][j]));
                }
            }
        }
        rows.push(row);
    }

    // if we want a thead use shift to get it
    thead = rows.shift();
    thead = $('<thead />').append(thead);
    table.append(thead);

    tbody = $('<tbody />');

    // add all the rows
    for (i = 0; i < rows.length; i = i + 1) {
        tbody.append(rows[i]);
    }

    table.append(tbody);

    return table;
};

var arrayToTableNoLinks = function (data, attrs) {
    var table = $('<table />'),
        thead,
        tbody,
        tfoot,
        rows = [],
        row,
        link,
        i,
        j;

    table.attr(attrs);

    table.addClass("ui celled table");

    // loop through all the rows, we will deal with tfoot and thead later
    for (i = 0; i < data.length; i = i + 1) {
        row = $('<tr />');
        link = data[i].pop();
        //row.on("click", load_link(link));
        //if (i != 0) {
          //row.attr('onclick', 'load_link("'+link+'");');
        //}

        for (j = 0; j < data[i].length; j = j + 1) {
            if (i === 0) {
                row.append($('<th style="text-align:center" />').html(data[i][j]));
            } else {
                if (j === 0) {
                    row.append($('<td />').html(data[i][j]));
                } else {
                    row.append($('<td style="text-align:center" />').html(data[i][j]));
                }
            }
        }
        rows.push(row);
    }

    // if we want a thead use shift to get it
    thead = rows.shift();
    thead = $('<thead />').append(thead);
    table.append(thead);

    tbody = $('<tbody />');

    // add all the rows
    for (i = 0; i < rows.length; i = i + 1) {
        tbody.append(rows[i]);
    }

    table.append(tbody);

    return table;
};

var arrayToTableReleaseNotes = function (data, attrs) {
    var table = $('<table />'),
        thead,
        tbody,
        tfoot,
        rows = [],
        row,
        link,
        i,
        j;

    table.attr(attrs);

    table.addClass("ui celled table");

    // loop through all the rows, we will deal with tfoot and thead later
    for (i = 0; i < data.length; i = i + 1) {
        row = $('<tr />');
        link = data[i].pop();
        //row.on("click", load_link(link));
        //if (i != 0) {
        //  row.attr('onclick', 'load_link("'+link+'");');
        //}

        for (j = 0; j < data[i].length; j = j + 1) {
            if (i === 0) {
                row.append($('<th />').html(data[i][j]));
            } else {
                //if (j === 0) {
                    row.append($('<td />').html(data[i][j]));
                //} else {
                    //row.append($('<td style="text-align:center" />').html(data[i][j]));
                //}
            }
        }
        rows.push(row);
    }

    // if we want a thead use shift to get it
    thead = rows.shift();
    thead = $('<thead />').append(thead);
    table.append(thead);

    tbody = $('<tbody />');

    // add all the rows
    for (i = 0; i < rows.length; i = i + 1) {
        tbody.append(rows[i]);
    }

    table.append(tbody);

    return table;
};

var insert_labs_table = function (tableID) {
  $.getJSON("data/labs.json", function( tableData ) {
    tableData.map(function(row){
      row[5] = "Software/mmWave Sensors/Industrial Toolbox/Labs/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTable(tableData, {}));
  });
};

var insert_labs_table2 = function (tableID) {
  $.getJSON("data/labs2.json", function( tableData ) {
    tableData.map(function(row){
      row[5] = "Software/mmWave Sensors/Industrial Toolbox/Labs/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTableNoLinks(tableData, {}));
  });
};

var insert_release_notes_table = function (tableID, datafile) {
  $.getJSON("data/release_notes.json", function( tableData ) {
    tableData.map(function(row){
      row[2] = "Software/mmWave Sensors/Industrial Toolbox/Labs/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTableReleaseNotes(tableData, {}));
  });
};

var insert_release_notes_table_prev = function (tableID, datafile) {
  $.getJSON("data/release_notes_prev.json", function( tableData ) {
    tableData.map(function(row){
      row[2] = "Software/mmWave Sensors/Industrial Toolbox/Labs/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTableReleaseNotes(tableData, {}));
  });
};

var insert_release_notes_table_prev2 = function (tableID, datafile) {
  $.getJSON("data/release_notes_prev2.json", function( tableData ) {
    tableData.map(function(row){
      row[2] = "Software/mmWave Sensors/Industrial Toolbox/Labs/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTableReleaseNotes(tableData, {}));
  });
};

var insert_experiments_table = function (tableID) {
  $.getJSON("data/experiments.json", function( tableData ) {
    tableData.map(function(row){
      row[2] = "Software/mmWave Sensors/Industrial Toolbox/Experiments/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTable(tableData, {}));
  });
};

var insert_ant_rad_patterns_table = function (tableID) {
  $.getJSON("data/antennas.json", function( tableData ) {
    tableData.map(function(row){
      row[7] = "Software/mmWave Sensors/Industrial Toolbox/Antenna Database/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTable(tableData, {}));
  });
};

var insert_ant_rad_patterns_table2 = function (tableID) {
  $.getJSON("data/modules.json", function( tableData ) {
    tableData.map(function(row){
      row[11] = "Software/mmWave Sensors/Industrial Toolbox/Antenna Database/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTable(tableData, {}));
  });
};

var load_link = function(path) {
  console.log("Loading link " + path);
  path = path.replace(/\(/, "\\(");
  path = path.replace(/\)/, "\\)");
  path = path.split('/');
  var tree = parent.$('#jstree').jstree(true);

  var id;
  var pathIndex = 0;

  var find_jstree_node_recursive_callback = function(node, status) {
    console.log("Opened node: " + tree.get_json(node).text);

    if (pathIndex >= path.length) {
      // Reached end of path, select desired node
      tree.select_node(node);
      return;
    }
    else {
      // Find next part of path in children of current node
      console.log("Searching for: " + path[pathIndex]);
      id = -1;
      $.each(tree.get_json(node).children, function(i, v) {
        if (v.text.search(new RegExp("^" + path[pathIndex],"i")) != -1) {
          id = v.id;
          return;
        }
      });
      if (id == -1) {
        console.log("ERROR: Failed to find: " + path[pathIndex]);
      }
      
      //Increment pathIndex before opening child node and callback function execution
      pathIndex = pathIndex + 1;

      // Open child node and then execute callback function
      tree.open_node(id, find_jstree_node_recursive_callback, false);
    }
  }

  // Find first node in TIREX tree (typically "Software")
  console.log("Searching for: " + path[pathIndex]);
  id = -1;
  $.each(tree.get_json(), function(i, v) {
    if (v.text.search(new RegExp("^" + path[pathIndex],"i")) != -1) {
      id = v.id;
      return;
    }
  });
  if (id == -1) {
    console.log("ERROR: Failed to find: " + path[pathIndex]);
  }

  //Increment pathIndex before opening first node and callback function execution
  pathIndex = pathIndex + 1;

  // Open first node and then execute callback function to recursively find rest of path
  tree.open_node(id, find_jstree_node_recursive_callback, false);
}
