####    CH47D   ####

aircraft.livery.init("Aircraft/ch47/Models/Liveries");
Ovolume=props.globals.getNode("/sim/sound/Ovolume",1);
var ramp = aircraft.door.new("/controls/ramp", 8);

#HelicopterEngine class 
# ie: var Eng = Engine.new(engine number,rotor_prop);
var Engine = {
    new : func(eng_num,rotor_prop){
        m = { parents : [Engine]};
        m.fdensity = getprop("consumables/fuel/tank/density-ppg");
        if(m.fdensity ==nil)m.fdensity=6.72;
        m.air_temp = props.globals.getNode("environment/temperature-degc",1);
        m.eng = props.globals.getNode("engines/engine["~eng_num~"]",1);
        m.rotor_rpm = props.globals.getNode(rotor_prop,1);
        m.running = m.eng.getNode("running",1);
        m.magneto = props.globals.getNode("controls/engines/engine["~eng_num~"]/magnetos",1);
        m.cutoff = props.globals.getNode("controls/engines/engine["~eng_num~"]/cutoff",1);
        m.rpm = m.eng.getNode("n2",1);
        m.fuel_pph=m.eng.getNode("fuel-flow_pph",1);
        m.oil_temp=m.eng.getNode("oil-temp-c",1);
        m.oil_temp.setDoubleValue(m.air_temp.getValue());
        m.fuel_pph.setDoubleValue(0);
        m.fuel_gph=m.eng.getNode("fuel-flow-gph",1);
        m.hpump=props.globals.getNode("systems/hydraulics/pump-psi["~eng_num~"]",1);
        m.hpump.setDoubleValue(0);
    return m;
    },
#### update ####
    update_eng : func(max_rpm){
        me.rpm.setValue(100/max_rpm * me.rotor_rpm.getValue() );
        var rpm =me.rpm.getValue();
        var hpsi =rpm;
        if(hpsi>60)hpsi = 60;
        me.hpump.setValue(hpsi);
        var OT= me.oil_temp.getValue();
        if(OT < rpm)OT+=0.01;
        if(OT > rpm)OT-=0.001;
        me.oil_temp.setValue(OT);
        },

    update_fuel : func(dt,gph){
        var cur_gph= gph * (me.rpm.getValue() * 0.01);
        var cur_pph = cur_gph * me.fdensity;
        me.fuel_gph.setDoubleValue(cur_gph);
       me.fuel_pph.setDoubleValue(cur_pph);
        var pph_used = (cur_pph/3600)*dt;
        me.eng.getNode("fuel-consumed-lbs").setValue(pph_used);
        fuel.update();
        },
#### check fuel cutoff , copy mixture setting to condition for turboprop ####
    condition_check :  func{
        if(me.cutoff.getBoolValue()){
            me.condition.setValue(0);
            me.running.setBoolValue(0);
        }else{
            me.condition.setValue(me.mixture.getValue());
        }
    }
};


var FHmeter = aircraft.timer.new("/instrumentation/clock/flight-meter-sec", 10);
FHmeter.stop();
var Eng = Engine.new(0,"rotors/main/rpm");

setlistener("/sim/signals/fdm-initialized", func {
    Ovolume.setValue(0.2);
    setprop("/instrumentation/clock/flight-meter-hour",0);
    settimer(update_systems,2);
    print("Aircraft Systems ... OK");
});

setlistener("/sim/current-view/internal", func(vw){
    if(vw.getValue()){
        Ovolume.setValue(0.3);
        }else{
        Ovolume.setValue(1.0);
        }
    },1,0);

setlistener("/gear/gear[1]/wow", func(gw){
    if(gw.getBoolValue()){
    FHmeter.stop();
    }else{
        FHmeter.start();
        }
},0,0);

setlistener("/sim/model/start-idling", func(idle){
    var run= idle.getBoolValue();
    if(run){
    Startup();
    }else{
    Shutdown();
    }
},0,0);

setlistener("/controls/ramp/position-norm", func(rmp){
    setprop("sim/multiplay/generic/float",rmp.getValue());
},1,0);

var Startup = func{
setprop("controls/electric/engine[0]/generator",1);
setprop("controls/electric/battery-switch",1);
setprop("controls/lighting/instrument-lights",1);
setprop("controls/lighting/nav-lights",1);
setprop("controls/lighting/beacon",1);
setprop("controls/lighting/strobe",1);
setprop("controls/engines/engine[0]/magnetos",3);
}

var Shutdown = func{
setprop("controls/electric/engine[0]/generator",0);
setprop("controls/electric/battery-switch",0);
setprop("controls/lighting/instrument-lights",0);
setprop("controls/lighting/nav-lights",0);
setprop("controls/lighting/beacon",0);
setprop("controls/engines/engine[0]/magnetos",0);
}

var flight_meter = func{
var fmeter = getprop("/instrumentation/clock/flight-meter-sec");
var fminute = fmeter * 0.016666;
var fhour = fminute * 0.016666;
setprop("/instrumentation/clock/flight-meter-hour",fhour);
}

var update_systems = func {
    flight_meter();
    Eng.update_eng(225);
    var dt = getprop("sim/time/delta-sec");
    Eng.update_fuel(dt,358);
    settimer(update_systems, 0);
}
