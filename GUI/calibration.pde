int input_box(float w, float h, int b, String value)
{
  int returnv;
  if (mouseX>w-15&&mouseY>h-5&&mouseX<(w+40)&&mouseY<(h+25)&&mousePressed)
  {
    returnv=b;
    box=true;
  } else
  {
    returnv=textbox;
  }
  textFont(text_font2);
  fill(255);
  noStroke();
  pushMatrix();
  translate(w, h);
  rectMode(CENTER);
  rect(10, 10, 50, 30);
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(15);
  if (returnv==b)
  {
    text(text[returnv]+(frameCount/10 % 2 == 0 ? "_" : ""), 10, 10);
  } else
  {
    text(text[b], 10, 10);
  }
  fill(0);
  textSize(20);
  text(value, 7, -20);
  popMatrix();

  return returnv;
}

void upload(int per)
{

  noStroke();
  pushMatrix();
  translate(width/1.2, height/1.4);
  if (mouseX>width/1.2-15&&mouseY>height/1.4-5&&mouseX<(width/1.2+40)&&mouseY<(height/1.4+25)&&mousePressed)
  {
    fill(0, 255, 0);
    uploading=true;
    uploaded=0;
  } else
  {
    fill(123);
  }
  rectMode(CENTER);
  rect(10, 10, 50, 30);
  fill(0);
  textAlign(CENTER, CENTER);
  textSize(20);
  text("UPLOAD", 10, 10);
  fill(255);
  rectMode(CENTER);
  rect(50, -80, 90, 30);

  fill(0, 255, 0);
  rectMode(CORNERS);
  if (per>75&&uploading==false)
  {
    per=95;
    rect(5, -95, per, -65);
  } else if (per>75)
    per=75;
  if (uploading==true)
  {
    rect(5, -95, per, -65);
    uploaded++;
    //arduino.write("2");
  }

  popMatrix();
  textFont(text_font);
}

