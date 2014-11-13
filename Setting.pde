class Settings {
        private String filePath = "";
        private PrintWriter output = null;


        public Settings(String filePath) {
                this.filePath = filePath;
        }


        public void load() {
                boolean isLoadIniPins = false;
                String line = null;
                BufferedReader reader = createReader(filePath); 

                while (true) {
                        try {
                                line = reader.readLine();
                        }catch(IOException e) {
                                e.printStackTrace();
                                line = null;
                                break;
                        }

                        if (line != null) {
                                if (line.length() > 0) {
                                        String[] pieces = split(line, " ");
                                        if (pieces[0].equals("HOST_IP:")) {
                                                HOST_IP = pieces[1];
                                        }
                                        if (pieces[0].equals("send_port:")) {
                                                send_port = int(pieces[1]);
                                        }
                                        if (pieces[0].equals("recieve_port:")) {
                                                recieve_port = int(pieces[1]);
                                        }
                                        if (pieces[0].equals("ScreenSize:")) {
                                                sw = int(pieces[1]);
                                                sh = int(pieces[2]);
                                        }
                                        if (pieces[0].equals("ScreenLocation:")) {
                                                screen_x = int(pieces[1]);
                                                screen_y = int(pieces[2]);
                                        }
                                        if (pieces[0].equals("minDepth:")) {
                                                minDepth = int(pieces[1]);
                                        }
                                        if (pieces[0].equals("maxDepth:")) {
                                                maxDepth = int(pieces[1]);
                                        }
                                        if (pieces[0].equals("mintouchDepth:")) {
                                                mintouchDepth = int(pieces[1]);
                                        }
                                        if (pieces[0].equals("maxtouchDepth:")) {
                                                maxtouchDepth = int(pieces[1]);
                                        }
                                        if (pieces[0].equals("threshold:")) {
                                                threshold = int(pieces[1]);
                                        }
                                        if (pieces[0].equals("sensor_area1:")) {
                                                area[0] =  new PVector(float(pieces[1]), float(pieces[2])) ;
                                        }
                                        if (pieces[0].equals("sensor_area2:")) {
                                                area[1] =  new PVector(float(pieces[1]), float(pieces[2])) ;
                                        }
                                        if (pieces[0].equals("sensor_area3:")) {
                                                area[2] =  new PVector(float(pieces[1]), float(pieces[2])) ;
                                        }
                                        if (pieces[0].equals("sensor_area4:")) {
                                                area[3] =  new PVector(float(pieces[1]), float(pieces[2])) ;
                                        }
                                        if (pieces[0].equals("sensor_area5:")) {
                                                area[4] =  new PVector(float(pieces[1]), float(pieces[2])) ;
                                        }
                                        if (pieces[0].equals("sensor_area6:")) {
                                                area[5] =  new PVector(float(pieces[1]), float(pieces[2])) ;
                                        }
                                        if (pieces[0].equals("sensor_area7:")) {
                                                area[6] =  new PVector(float(pieces[1]), float(pieces[2])) ;
                                        }
                                        if (pieces[0].equals("sensor_area8:")) {
                                                area[7] =  new PVector(float(pieces[1]), float(pieces[2])) ;
                                        }
                                }
                        }else {
                                //error occured or file is empty
                                break;
                        }
                }


        }

        public void save() {
                output = createWriter(filePath);
                
                output.println("HOST_IP: " + HOST_IP);
                output.println("");
                
                output.println("send_port: " + send_port);
                output.println("");
                
                output.println("recieve_port: " + recieve_port);
                output.println("");
                
                //write With and Height of screen
                output.println("ScreenSize: " + sw + " " + sh);
                output.println("");

                //write location of screen
                output.println("ScreenLocation: " + screen_x + " " + screen_y);
                output.println("");

                //write the minDepth
                output.println("minDepth: " + minDepth);
                output.println("");
                
                //write the maxDepth
                output.println("maxDepth: " + maxDepth);
                output.println("");
                
                //write the mintouchDepth
                output.println("mintouchDepth: " + mintouchDepth);
                output.println("");
                
                //write the maxtouchDepth
                output.println("maxtouchDepth: " + maxtouchDepth);
                output.println("");

                //write the maxDepth
                output.println("threshold: " + threshold);
                output.println("");
                
                output.println("# each sensor area points are blow:");
                output.println("");

                // Write every sensor area point's location
                for (int i=0; i<area.length; i++) {
                        output.println("sensor_area"+ (i+1) +": " + area[i].x + " "+ area[i].y);
                        output.println("");
                }

                output.flush(); // Writes the remaining data to the file
                output.close(); // Finishes the file
        }
}


void loadInitialSettings() {
  //loading initial file and pins' location file
  settings = new Settings(dataPath("settings.txt"));
  println("loading settings.txt");
  TIP_MSG = "loading settings ......";
  settings.load();
  println("loading SUCESS!!");
}


