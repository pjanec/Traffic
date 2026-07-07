# 15-reroute (B3 behavioral fixture)

No SUMO golden: the blocking obstacle is a live external input (DESIGN.md "Two futures" / Group B).
Validation is behavioral (`RungB3RerouteTests`): a vehicle routed the top path SA AB BD DE, with a
persistent obstacle injected on BD, must reroute (B2 router, avoiding BD) after the threshold and
divert to the bottom path SA AC CD DE, reaching DE without entering BD. The diamond net is shared
with scenarios/_fixtures/routing-diamond (top AB/BD len 505.07 each vs bottom AC/CD 634.63 each).
