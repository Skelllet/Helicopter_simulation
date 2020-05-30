using System;
using NetTopologySuite.Geometries;
using OSMLSGlobalLibrary;
using OSMLSGlobalLibrary.Map;
using OSMLSGlobalLibrary.Modules;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Threading;
using System.Globalization;

namespace Helicopter_simulation
{
    public static class CoordinateExtension
    {
        public static (double la, double lo) toLatLon(this Coordinate coord)
        {
            var la = yToLat(coord.Y);
            var lo = xToLon(coord.X);
            return (la, lo);
        }

        private static readonly double R_MAJOR = 6378137.0;
        private static readonly double R_MINOR = 6356752.3142;
        private static readonly double RATIO = R_MINOR / R_MAJOR;
        private static readonly double ECCENT = Math.Sqrt(1.0 - (RATIO * RATIO));
        private static readonly double COM = 0.5 * ECCENT;

        private static readonly double RAD2Deg = 180.0 / Math.PI;
        private static readonly double PI_2 = Math.PI / 2.0;

        public static double xToLon(double x)
        {
            return RadToDeg(x) / R_MAJOR;
        }

        public static double yToLat(double y)
        {
            double ts = Math.Exp(-y / R_MAJOR);
            double phi = PI_2 - 2 * Math.Atan(ts);
            double dphi = 1.0;
            int i = 0;
            while ((Math.Abs(dphi) > 0.0000001) && (i < 15))
            {
                double con = ECCENT * Math.Sin(phi);
                dphi = PI_2 - 2 * Math.Atan(ts * Math.Pow((1.0 - con) / (1.0 + con), COM)) - phi;
                phi += dphi;
                i++;
            }

            return RadToDeg(phi);
        }

        private static double RadToDeg(double rad)
        {
            return rad * RAD2Deg;
        }
    }

    public static class PointExtension
    {
        public static double distance(this Point p1, Point p2) => Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        public static double distance(this Point p1, Coordinate p2) => Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));

        public static void Move(this Point p, Coordinate direction)
        {
            p.X += direction.X;
            p.Y += direction.Y;
        }

        public static void Move(this Point p, Coordinate direction, double speed)
        {
            double MinimumDirection(double s, double d) =>
                Math.Min(speed, Math.Abs(s - d)) * Math.Sign(d - s);

            p.X += MinimumDirection(p.X, direction.X);
            p.Y += MinimumDirection(p.Y, direction.Y);
        }
    }

    static class Rand
    {
        private static Random rand = new Random();

        public static int GenerateInRange(int min, int max) =>
            (int)Math.Round(min - 0.5 + rand.NextDouble() * (max - min + 1));

        public static Coordinate GenerateNext((int leftX, int rightX, int downY, int upY) map) =>
            new Coordinate(GenerateInRange(map.leftX, map.rightX), GenerateInRange(map.downY, map.upY));

        public static T RandomElement<T>(this ICollection<T> q)
        {
            return q.Skip(rand.Next(q.Count())).FirstOrDefault();
        }
        public static bool MayBe(double p = 0.5)
        {
            return rand.NextDouble() <= p;
        }
    }


    public class WindMap
    {
        private readonly Dictionary<(int, int), Coordinate> _coordinatesToWindVec;
        public WindMap()
        {
            _coordinatesToWindVec = new Dictionary<(int, int), Coordinate>();
        }

        public void Insert(int x, int y, double U, double V)
        {
            _coordinatesToWindVec[(y, x)] = new Coordinate(Convert.ToInt32(U), Convert.ToInt32(V));
        }

        public Coordinate GetWindDirection(Coordinate position)
        {
            var (la, lo) = position.toLatLon();
            var y = (int)Math.Round(la);
            var x = (int)Math.Abs(Math.Round(lo));

            return _coordinatesToWindVec[(y, x)];
        }
    }



    public class Helicopter : OSMLSModule
    {
        private WindMap _windMap;
       // private List<(double, double)> _cities;
        List<(double, double)> _cities = new List<(double , double)>();

        

        private readonly (int, int, int, int) _allMap = (0, 18628621 * 2, -15000000, 15000000);
        private int MultirotorCounter;



        public (int leftX, int rightX, int downY, int upY) map;

        List<(double lat, double lon)> helipadLatLon = new List<(double lat, double lon)>();

        List<(helicopter, Coordinate)> helicopters = new List<(helicopter, Coordinate)>();

        List<Storm> storms = new List<Storm>();
        const int maximumHelicopter = 300;

        protected override void Initialize()
        {
            _windMap = new WindMap();
            var csvWindData = File.ReadLines("./current-wind.csv");
            foreach (var line in csvWindData.Skip(1))
            {
                if (line.Length == 0) continue;
                // U,V,la,lo;
                var data = line.Split(',').Select(s =>
                {
                    double.TryParse(s.Replace('.', ','), out var d);
                    return d;
                }).ToArray();
                double la = data[2], lo = data[3], U = data[0], V = data[1];
                _windMap.Insert((int)lo, (int)la, U * 100, V * 100);
            }
            

            const string textFileCities = "cities.dat";

            if (File.Exists(textFileCities))
            {
                var lines = File.ReadAllLines(textFileCities);
                foreach (var line in lines)
                {
                    var row = line.Split(",");
                    CultureInfo temp_culture = Thread.CurrentThread.CurrentCulture;
                    Thread.CurrentThread.CurrentCulture = CultureInfo.CreateSpecificCulture("en-US");
                    _cities.Add((Convert.ToDouble(row[0]), Convert.ToDouble(row[1])));
                    Thread.CurrentThread.CurrentCulture = temp_culture;

                    Console.WriteLine(row[0] + " " + row[1]);

                }
            }
            else
            {
                throw new Exception("Data file not found");
            }


            


            MapObjects.Add(new Multirotor(new Coordinate(4940278, 6233593), _windMap));
            MultirotorCounter = 1;


            var rand = new Random();

            const string textFile = "helipad.dat";

            if (File.Exists(textFile))
            {
                var lines = File.ReadAllLines(textFile);
                foreach (var line in lines)
                {
                    var row = line.Split(",");
                    CultureInfo temp_culture = Thread.CurrentThread.CurrentCulture;
                    Thread.CurrentThread.CurrentCulture = CultureInfo.CreateSpecificCulture("en-US");
                    helipadLatLon.Add((Convert.ToDouble(row[0]), Convert.ToDouble(row[1])));
                    Thread.CurrentThread.CurrentCulture = temp_culture;

                    Console.WriteLine(row[0] + " " + row[1]);

                }
            }
            else
            {
                throw new Exception("Data file not found");
            }


            for (int i = 0; i < helipadLatLon.Count; i++)
            {
                var helipadCoordinate = MathExtensions.LatLonToSpherMerc(helipadLatLon[i].lat, helipadLatLon[i].lon);
                MapObjects.Add(new Helipad(helipadCoordinate, 0));
            }


            for (int i = 0; i < 20; i++)
            {
                AddStorm();
            }


            for (int i = 0; i < 50; i++)
            {
                AddHelicopter();
            }

        }

        public void AddHelicopter()
        {
            var sourceHelipad = helipadLatLon[new Random().Next(helipadLatLon.Count)];
            var destHelipad = helipadLatLon[new Random().Next(helipadLatLon.Count)];

            while (destHelipad == sourceHelipad)
            {
                destHelipad = helipadLatLon[new Random().Next(helipadLatLon.Count)];
            }

            var helipadCoordinate = MathExtensions.LatLonToSpherMerc(sourceHelipad.Item1, sourceHelipad.Item2);
            var destHelipadCoordinate = MathExtensions.LatLonToSpherMerc(destHelipad.Item1, destHelipad.Item2);

            var helicopter = new helicopter(helipadCoordinate, 220);

            helicopters.Add((helicopter, destHelipadCoordinate));
            MapObjects.Add(helicopter);
        }


        /// <summary>
        /// Вызывается постоянно, здесь можно реализовывать логику перемещений и всего остального, требующего времени.
        /// </summary>
        /// <param name="elapsedMilliseconds">TimeNow.ElapsedMilliseconds</param>
        public override void Update(long elapsedMilliseconds)
        {
            try
            {
                foreach (var multirotor in MapObjects.GetAll<Multirotor>())
                {
                    multirotor.Move();
                }
            }
            catch (KeyNotFoundException)
            {
                /* pass */
            }

            

            if (Rand.MayBe(0.3) && MultirotorCounter < 300)
            {
                var coord = Rand.GenerateNext(_allMap);
                MapObjects.Add(new Multirotor(coord, _windMap));
                Console.WriteLine("На координатах {0}, {1} был запущен мультикоптер", coord.X, coord.Y);
                MultirotorCounter += 1;
            }
           
            else
            {
                try
                {
                    MapObjects.GetAll<Multirotor>().Where(multirotor => multirotor.Declining()).ToList()
                        .ForEach(multirotor =>
                        {
                            MapObjects.Remove(multirotor);
                            var coord = multirotor.Coordinate;
                            MultirotorCounter -= 1;
                            Console.WriteLine("На координатах {0}, {1} приземлился мультикоптер", coord.X, coord.Y);
                        });
                }
                catch (KeyNotFoundException)
                {
                    /* pass */
                }

               
            }



            if (helicopters.Count < maximumHelicopter)
            {
                AddHelicopter();
                Console.WriteLine("[Вертолет " + helicopters[helicopters.Count - 1].Item1.X + " " + helicopters[helicopters.Count - 1].Item1.Y + "]");
                Console.WriteLine("Началась посадка");
                Console.WriteLine("Пассажиры зарегистрированы");
                Console.WriteLine("Пассажиры на местах");
                Console.WriteLine("Вертолет вылетел из " + helicopters[helicopters.Count - 1].Item1.X + " " + helicopters[helicopters.Count - 1].Item1.Y);
            }

            var arrivedHelicopters = helicopters.Where(helicopter => (Math.Abs(helicopter.Item1.X - helicopter.Item2.X) < helicopter.Item1.Speed || Math.Abs(helicopter.Item1.Y - helicopter.Item2.Y) < helicopter.Item1.Speed)).ToList();

            foreach (var helicopter in arrivedHelicopters)
            {
                MapObjects.Remove(helicopter.Item1);
                helicopters.Remove(helicopter);
                Console.WriteLine("[Вертолет " + helicopter.Item1.X + " " + helicopter.Item1.Y + "]");
                Console.WriteLine("Вертолет прибыл в " + helicopter.Item1.X + " " + helicopter.Item1.Y);
                Console.WriteLine("Началась высадка пассажиров");
                Console.WriteLine("Пассажиры высажены");
            }


            Console.WriteLine("Всего вертолетов в небе: " + helicopters.Count);

            foreach (var helicopter in helicopters)
            {
                var realhelicopter = helicopter.Item1;
                var realDest = helicopter.Item2;
                realhelicopter.FlyToHelipad(realDest, storms);
            }

            foreach (var storm in storms)
            {
                storm.X -= storm.Speed;
                foreach (var helicopter in helicopters)
                {

                }
            }
        }



        public void AddStorm()
        {
            var rand = new Random();

            var coord = MathExtensions.LatLonToSpherMerc(rand.Next(-70, 70), rand.Next(-70, 70));
            var storm = new Storm(coord, 10);
            MapObjects.Add(storm);

            storms.Add(storm);
        }
    }

    #region объявления класса, унаследованного от точки, объекты которого будут иметь уникальный стиль отображения на карте

    /// <summary>
    /// Вертолет, умеющий летать вверх-вправо с заданной скоростью.
    /// </summary>
    [CustomStyle(
        @"new ol.style.Style({
            image: new ol.style.Circle({
                opacity: 1.0,
                scale: 1.0,
                radius: 4,
                fill: new ol.style.Fill({
                    color: 'rgba(222, 0, 0, 1)'
                }),
                stroke: new ol.style.Stroke({
                    color: 'rgba(0, 0, 0, 0.4)',
                    width: 2
                }),
            })
        });
        ")] // Переопределим стиль всех объектов данного класса, сделав вертолет красным, используя атрибут CustomStyle.
    class helicopter : Point // Унаследуем данный данный класс от стандартной точки.
    {
        /// <summary>
        /// Скорость вертолета.
        /// </summary>
        public double Speed { get; }
        public Coordinate coordinate;

        /// <summary>
        /// Конструктор для создания нового объекта.
        /// </summary>
        /// <param name="coordinate">Начальные координаты.</param>
        /// <param name="speed">Скорость.</param>
        public helicopter(Coordinate coordinate, double speed) : base(coordinate)
        {
            Speed = speed;
            this.coordinate = coordinate;
        }

        /// <summary>
        /// Двигает вертолет.
        /// </summary>
        internal void FlyToHelipad(Coordinate mymap, List<Storm> storms)
        {
            double eps = 2 * Speed;


            if (coordinate.X < mymap.X)
            {
                X += Speed;
            }
            if (coordinate.X > mymap.X)
            {
                X -= Speed;
            }
            if (coordinate.Y < mymap.Y)
            {
                Y += Speed;
            }
            if (coordinate.Y > mymap.Y)
            {
                Y -= Speed;
            }

        }
    }

    /// <summary>
    /// облако
    /// </summary>
    [CustomStyle(
        @"new ol.style.Style({
            image: new ol.style.Circle({
                opacity: 1.0,
                scale: 1.0,
                radius: 20,
                fill: new ol.style.Fill({
                    color: 'rgba(155, 218, 250, 1)'
                }),
                stroke: new ol.style.Stroke({
                    color: 'rgba(0, 0, 0, 0.4)',
                    width: 1
                }),
            })
        });
        ")]
    class Storm : Point
    {
        public Coordinate coordinate;
        public double Speed { get; }


        public Storm(Coordinate coordinate, double speed) : base(coordinate)
        {
            this.coordinate = coordinate;
            Speed = speed;

        }
    }

    /// <summary>
    /// Вертодром
    /// </summary>
    [CustomStyle(
        @"new ol.style.Style({
            image: new ol.style.Circle({
                opacity: 1.0,
                scale: 1.0,
                radius: 6,
                fill: new ol.style.Fill({
                    color: 'rgba(17, 255, 0, 1)'
                }),
                stroke: new ol.style.Stroke({
                    color: 'rgba(0, 0, 0, 0.4)',
                    width: 1
                }),
            })
        });
        ")]
    class Helipad : Point
    {
        public Coordinate coordinate;
        public double Speed { get; }


        public Helipad(Coordinate coordinate, double speed) : base(coordinate)
        {
            this.coordinate = coordinate;
            Speed = speed;

        }
    }


   


    [CustomStyle(
        @"new ol.style.Style({
            image: new ol.style.Circle({
                opacity: 1.0,
                scale: 1.0,
                radius: 2,
                fill: new ol.style.Fill({
                    color: 'rgba(219, 240, 115, 1)'
                }),
                stroke: new ol.style.Stroke({
                    color: 'rgba(0, 0, 0, 0.4)',
                    width: 1
                }),
            })
        });
        ")]
    class Multirotor : Point
    {
        private readonly WindMap _windMap;
        private int _countSteps;
        public Multirotor(Coordinate coordinate, WindMap windMap) : base(coordinate)
        {
            _windMap = windMap;
            _countSteps = Rand.GenerateInRange(500, 700);
        }

        public void Move()
        {
            _countSteps = Math.Max(0, _countSteps - 1);
            var direction = _windMap.GetWindDirection(this.Coordinate);

            this.Move(direction);
        }

        public bool Declining()
        {
            return _countSteps == 0;
        }
    }

    

        #endregion
    
}

    

