package org.usfirst.frc.team839.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldLayout {
	FieldLayout (String layout){
		this.ourSwitch = layout.substring(0, 1);
		this.scale = layout.substring(1,2);
		this.theirSwitch = layout.substring(2);
}
private String ourSwitch = null;
private String theirSwitch = null;
private String scale = null;
public boolean isOurSwitch(String side){
	return side.equalsIgnoreCase("B")||this.ourSwitch.equalsIgnoreCase(side);
}
public boolean isTheirSwitch(String side){
	return this.theirSwitch.equalsIgnoreCase(side);
}
public boolean isOurScale(String side){
	return this.scale.equalsIgnoreCase(side);
}
public String getOurSwitch() {
	return ourSwitch;
}
public void setOurSwitch(String ourSwitch) {
	this.ourSwitch = ourSwitch;
}
public String getTheirSwitch() {
	return theirSwitch;
}
public void setTheirSwitch(String theirSwitch) {
	this.theirSwitch = theirSwitch;
}
public String getScale() {
	return scale;
}
public void setScale(String scale) {
	this.scale = scale;
}
}
