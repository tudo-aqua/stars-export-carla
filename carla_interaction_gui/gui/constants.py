# Official CARLA maps from the docs (non-layered + layered "_Opt")
ALLOWED_NON_LAYERED_MAPS = [
    "Town01",
    "Town02",
    "Town03",
    "Town04",
    "Town05",
    "Town10HD",
]
ALLOWED_PARKED_MAPS = [
    "Town03",
    "Town05",
    "Town10HD",
]
ALLOWED_CARLA_MAPS = ALLOWED_NON_LAYERED_MAPS

# All official CARLA drivable vehicle blueprints from the docs (cars, vans,
# trucks, buses, motorcycles, bicycles), selectable as the ego vehicle for
# manual driving. Keep "vehicle.lincoln.mkz_2017" first: it's the
# long-standing hardcoded default and must stay the fallback.
ALLOWED_EGO_VEHICLES = [
    "vehicle.lincoln.mkz_2017",
    # Cars
    "vehicle.lincoln.mkz_2020",
    "vehicle.audi.a2",
    "vehicle.audi.tt",
    "vehicle.audi.etron",
    "vehicle.bmw.grandtourer",
    "vehicle.chevrolet.impala",
    "vehicle.citroen.c3",
    "vehicle.dodge.charger_2020",
    "vehicle.dodge.charger_police",
    "vehicle.dodge.charger_police_2020",
    "vehicle.ford.crown",
    "vehicle.ford.mustang",
    "vehicle.jeep.wrangler_rubicon",
    "vehicle.mercedes.coupe",
    "vehicle.mercedes.coupe_2020",
    "vehicle.micro.microlino",
    "vehicle.mini.cooper_s",
    "vehicle.mini.cooper_s_2021",
    "vehicle.nissan.micra",
    "vehicle.nissan.patrol",
    "vehicle.nissan.patrol_2021",
    "vehicle.seat.leon",
    "vehicle.tesla.model3",
    "vehicle.toyota.prius",
    # Vans
    "vehicle.ford.ambulance",
    "vehicle.mercedes.sprinter",
    "vehicle.volkswagen.t2",
    "vehicle.volkswagen.t2_2021",
    # Trucks
    "vehicle.carlamotors.carlacola",
    "vehicle.carlamotors.european_hgv",
    "vehicle.carlamotors.firetruck",
    "vehicle.tesla.cybertruck",
    # Buses
    "vehicle.mitsubishi.fusorosa",
    # Motorcycles
    "vehicle.harley-davidson.low_rider",
    "vehicle.kawasaki.ninja",
    "vehicle.vespa.zx125",
    "vehicle.yamaha.yzf",
    # Bicycles
    "vehicle.bh.crossbike",
    "vehicle.diamondback.century",
    "vehicle.gazelle.omafiets",
]
