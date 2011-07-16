/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2010 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Tracking/LiveTrack24.hpp"
#include "Replay/IGCParser.hpp"
#include "IO/FileLineReader.hpp"
#include "Net/Init.hpp"
#include "ConsoleJobRunner.hpp"
#include "DateTime.hpp"
#include "Units/Units.hpp"
#include "Engine/Navigation/Geometry/GeoVector.hpp"

#include <cstdio>

using namespace LiveTrack24;

static void
TestTracking(const char *filename)
{
  printf("Opening IGC file \"%s\"\n", filename);
  FileLineReaderA reader(filename);
  if (reader.error()) {
    fprintf(stderr, "Failed to open %s\n", filename);
    return;
  }

  SessionID session = GenerateSessionID();
  printf("Generated session id: %u\n", session);

  bool result;
  unsigned pid = 1;
  printf("Starting tracking ... ");
  result = StartTracking(session, _T("testguest"), _T(""), 10, VT_GLIDER, _T("Hornet"));
  printf(result ? "done\n" : "failed\n");
  if (!result)
    return;

  printf("Sending positions ");
  char *line;
  BrokenDate date = BrokenDateTime::NowUTC();
  bool first = true;
  IGCFix last;
  for (int i = 1; (line = reader.read()) != NULL; i++) {
    if (i % 10 == 0) {
      putchar('.');
      fflush(stdout);
    }

    IGCFix fix;
    if (!IGCParseFix(line, fix))
      continue;

    if (first) {
      first = false;
      last = fix;
      continue;
    }

    const BrokenTime time = BrokenTime::FromSecondOfDay(fix.time);
    BrokenDateTime datetime(date.year, date.month, date.day,
                            time.hour, time.minute, time.second);

    GeoVector gv = last.location.distance_bearing(fix.location);
    result = SendPosition(session, ++pid, fix.location, fix.pressure_altitude,
                          Units::ToUserUnit(gv.Distance / (fix.time - last.time),
                                            unKiloMeterPerHour), gv.Bearing, datetime);

    if (!result)
      break;

    last = fix;
  }
  printf(result ? "done\n" : "failed\n");

  printf("Stopping tracking ... ");
  result = EndTracking(session, ++pid);
  printf(result ? "done\n" : "failed\n");
}

int
main(int argc, char *argv[])
{
  assert(argc >= 2);

  Net::Initialise();

  LiveTrack24::SetTest(true);
  TestTracking(argv[1]);

  Net::Deinitialise();

  return 0;
}
