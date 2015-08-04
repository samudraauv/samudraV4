void header()
{
  fill(0);
  textSize(50);
  textAlign(CENTER, CENTER);
  textFont(text_font);
  text("SAMUDRA", width/2, height/20);
  stroke(0);
  strokeWeight(4);
  line(width/2-130, height/20+30, width/2+130, height/20+30);
}



void battery(float level)
{
  if (level>37.00)
    level=37;

  if (level<0.00)
    level=0.00;
  fill(255);
  stroke(0);
  strokeWeight(2);
  pushMatrix();
  translate(width/30, height/5);
  beginShape();
  vertex(15, 0);
  vertex(30, 0);
  vertex(30, 10);
  vertex(45, 10);
  vertex(45, 60);
  vertex(0, 60);
  vertex(0, 10);
  vertex(15, 10);
  vertex(15, 0);
  endShape();

  fill(0, 255, 0);
  noStroke();
  rectMode(CORNERS);
  rect(3, 58, 43, 58-level);

  fill(0);
  textAlign(CENTER, CENTER);
  textSize(15);
  text("BATTERY--", 25, -30);
  text((level/37.0)*100 + " " + "%", 25, -15);
  fill(0, 255, 0);

  popMatrix();
}

void temp(float level)
{
  noStroke();
  pushMatrix();
  translate(width/1.1, height/5);
  fill(255);
  rectMode(CENTER);
  rect(20, 30, 80, 25);
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(15);
  text(level + " " + "C", 20, 30);
  textSize(18);
  text("TEMPERTAURE", 5, 0);
  popMatrix();
}

void status(String print, int value)
{
  noStroke();
  pushMatrix();
  translate(width/30, height/2.8);
  fill(255);
  rectMode(CORNERS);
  rect(0, 30, 300, 0);
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(15);
  text("STATUS OF THE VEHICLE", 140, -12);
  textSize(14);
  if (value<=10&&value>5)
  {
    fill(0, 255, 0);
    text("RUNING STATE-" +" " + value+" "+print, 140, 15);
  } else if (value>10)
  {
    fill(255, 0, 0);
    text("ERROR STATE-" +" " + value+" "+print, 140, 15);
  } else
  {
    fill(0, 0, 255);
    text("INTIAL STATE-" +" " + value+" "+ print, 140, 15);
  }
  popMatrix();
}

void eprom(int value1, int value2)
{
  pushMatrix();
  translate(width/30, height/2.1);
  fill(255);
  rectMode(CORNERS);
  rect(0, 30, 300, 0);
  fill(255);
  textAlign(CENTER, CENTER);
  textSize(15);
  text("EPROM STATUS", 140, -12);
  textSize(14);
  fill(255, 0, 0);
  text("TEMP-" + " " + value1 +"," + " " + "CODE-" + " " + value2, 140, 15);
  popMatrix();
}

void timer()
{
  min=int(millis()/60000);
  if (sec>59)
  {
    i=i+1;
  }
  sec=int(millis()/1000)-(60*i);
  pushMatrix();
  translate(width/1.1, height/8);
  textAlign(CENTER, CENTER);
  textSize(25);
  fill(0);
  text("DATE--" + day() + "/" + month()+"/"+year(), -30, -20);
  text("COUNT--" + min + ":" + sec, -5, 0);
  popMatrix();
}

void pointer()
{
  noFill();
  rectMode(CENTER);

  if (mouseX>5&&mouseY>5&&mouseX<width-5&&mouseY<height-5)
    fill(255, 0, 0);
  rect(mouseX, mouseY, 10, 10);
}

