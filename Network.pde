void receive( byte[] data, String ip, int port ) {  // <-- extended handler
  data = subset(data, 0, data.length-2);
  String message = new String( data );
  
  println( "receive: \""+message+"\" from "+ip+" on port "+port );
  if(data.length>2) {
    client_center.clear();
    String input = new String(data);
    if(input!="") {
      //input = input.substring(0, input.indexOf("\n"));
      tdata = float(split(input, ','));
    }
    println(tdata);
    for(int i=0; i<tdata.length; i+=2) {
      println(i+"_"+tdata[i]+","+tdata[i+1]);
      client_center.add(new PVector(tdata[i],tdata[i+1]));
    }
  }
}

void sendInteraction(ArrayList<PVector> pt) {
  String msg = "";
  for (int k = 0; k< pt.size (); k++) {
    PVector tmp_p = pt.get(k);
    fill(200, 100, 50, 100);
    ellipse(tmp_p.x, tmp_p.y, 100, 100);
    if(msg=="") {
      msg = tmp_p.x+ ","+tmp_p.y ;
    }else {
      msg = msg+ "," +tmp_p.x+ ","+tmp_p.y ;
    }
  }
  msg+="\n";
  udp.send( msg, HOST_IP, send_port );
}

void drawClientInteractive(ArrayList<PVector> pt) {
  for (int k = 0; k< pt.size (); k++) {
    PVector tmp_p = pt.get(k);
    fill(50, 100, 200, 100);
    ellipse(sw-tmp_p.x, sh-tmp_p.y, 100, 100);
  }
}
