#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h> 

//czas i szybkość
#define TIME_STEP 32
#define MAX_SPEED 6.28/2

//tolerancje zmiennych
#define DISTANCE_TOLERANCE 0.005
#define AXIS_TOLERANCE 0.001
#define TARGET_AXIS_TOLERANCE 0.000005
#define ANGLE_TOLERANCE 0.000001
#define END_TARGET_TOLERANCE 0.005

//szybkość skrętów
#define ROTATE_SPEED MAX_SPEED/3
#define SLOW_ROTATE_SPEED MAX_SPEED/50
#define ONE_WHEEL_ROTATE_SPEED 0.9*MAX_SPEED

//parametry labiryntu
#define SIZE_STEP 0.2
#define WORLD_SIZE 1.6
#define OBSTACLE_VALUE 80

//współrzędne końca labiryntu
#define END_TARGET_X 0.05
#define END_TARGET_Z -0.05

//wykrycie przeszkód
static bool center_obstacle;
static bool right_obstacle;
static bool left_obstacle;

//prędkość silników
static double left_speed = 0;
static double right_speed = 0;

//wspołrzędne kolejnego punktu
static double target_x = -0.75;
static double target_z = 0.65;

//osiągnięcie punktu
static bool target = false;

//określenie osi po których się porusza
static double target_axis_x = 1.0;
static double target_axis_z = -1.0;
static bool go_x = false;

//zmienne skrętów
static bool turn_right = false;
static bool turn_left = false;
static bool turn_back = false;

//zmienna dotarcia do celu
static bool end = false;

//tryb jazdy
  //1 - mapowanie
  //2 - algorytm propagacji fali
  //3 - algorytm Dijkstry
#define TYPE 3

//mapowanie labiryntu
#define WEST 1
#define SOUTH 2
#define EAST 4
#define NORTH 8
#define PLIK "map.txt"
#define GRAFN 256
#define EDGEN 538
#define ENDGRAPH 136
static int map[256];
static int road[100];
static int countR = 0;
static int step = 0;
static int points = 256;
static int index = 16;

//graf 
struct Graph
{
  struct Node* head[GRAFN];
};

//wierzchołek
struct Node
{
  int dest;
  int val;
  struct Node* next;	
};

//krawędź
struct Edge
{
  int src, dest, val;	
};

//graf
static struct Graph* graph;


//kat odchylenia od obranego kierunku
double getAlpha(double north_x, double north_z) 
{
  double alphaRad = atan2(north_x, north_z);
  double alpha = (alphaRad - 1.5708) / M_PI * 180.0;
  
  if (alpha < 0.0)
  {
    alpha = alpha + 360.0;
  }
  
  return alpha;
}

//skręt w prawo
void turnRight(double axis)
{
  left_speed = ROTATE_SPEED;
  right_speed = -ROTATE_SPEED;
  
  double axis_target;
  
  if (go_x) //jade po x
  {
    axis_target = -target_axis_x;
  }
  else //jade po z
  {
    axis_target = -target_axis_z;
  }
  
  //zwolnij gdy blisko poprawnego kierunku
  if (axis >= (axis_target - AXIS_TOLERANCE) && axis <= (axis_target + AXIS_TOLERANCE))
  {
    left_speed = SLOW_ROTATE_SPEED;
    right_speed = -SLOW_ROTATE_SPEED;
  }
  
  //dobry kierunek
  if (axis >= (axis_target - TARGET_AXIS_TOLERANCE) && axis <= (axis_target + TARGET_AXIS_TOLERANCE))
  {
    left_speed = 0;
    right_speed = 0;
    turn_right = false;
  }
}

//skręt w lewo
void turnLeft(double axis)
{
  left_speed = -ROTATE_SPEED;
  right_speed = ROTATE_SPEED;
  
  double axis_target;
  
  if (go_x) //jade po x
  {
    axis_target = -target_axis_x;
  }
  else //jade po z
  {
    axis_target = -target_axis_z;
  }
  
  //zwolnij gdy blisko poprawnego kierunku
  if (axis >= (axis_target - AXIS_TOLERANCE) && axis <= (axis_target + AXIS_TOLERANCE))
  {
    left_speed = -SLOW_ROTATE_SPEED;
    right_speed = SLOW_ROTATE_SPEED;
  }
  
  //dobry kierunek
  if (axis >= (axis_target - TARGET_AXIS_TOLERANCE) && axis <= (axis_target + TARGET_AXIS_TOLERANCE))
  {
    left_speed = 0;
    right_speed = 0;
    turn_left = false;
  }
}

//zawracanie
void turnBack(double axis)
{
  left_speed = ROTATE_SPEED;
  right_speed = -ROTATE_SPEED;
  
  double axis_target;
  
  if (go_x) //jade po x
  {
    axis_target = -target_axis_x;
  }
  else //jade po z
  {
    axis_target = -target_axis_z;
  }
  
  //zwolnij gdy blisko poprawnego kierunku
  if (axis >= (axis_target - AXIS_TOLERANCE) && axis <= (axis_target + AXIS_TOLERANCE))
  {
    left_speed = SLOW_ROTATE_SPEED;
    right_speed = -SLOW_ROTATE_SPEED;
  }
  
  //dobry kierunek
  if (axis >= (axis_target - TARGET_AXIS_TOLERANCE) && axis <= (axis_target + TARGET_AXIS_TOLERANCE))
  {
    left_speed = 0;
    right_speed = 0;
    turn_back = false;
  }
}

//jazda przed siebie
void goStraight(double axis_x, double axis_z)
{
  double left = MAX_SPEED;
  double right = MAX_SPEED;

  double alpha; //kąt odchylenia od obranego kierunku

  if (go_x) //jade po x
  {
    if (target_axis_x == 1.0) //jade po +x
    {
      alpha = getAlpha(-axis_z, axis_x);
      
      //korekta ruchu
      if (alpha > ANGLE_TOLERANCE && alpha < 180) 
      {
        left = ONE_WHEEL_ROTATE_SPEED;
      } 
      else if (alpha < (360+ANGLE_TOLERANCE)) 
      {
        right = ONE_WHEEL_ROTATE_SPEED;
      }
    }
    else //jade po -x
    { 
      alpha = getAlpha(axis_z, axis_x);
      
      //korekta ruchu
      if (alpha > ANGLE_TOLERANCE && alpha < 180) 
      {
        right = ONE_WHEEL_ROTATE_SPEED;
      } 
      else if (alpha < (360+ANGLE_TOLERANCE)) 
      {
        left = ONE_WHEEL_ROTATE_SPEED;
      } 
    }
  }
  else //jade po z
  {
    if (target_axis_z == 1.0) //jade po +z
    {
      alpha = getAlpha(-axis_x, axis_z);
      
      //korekta ruchu
      if (alpha > ANGLE_TOLERANCE && alpha < 180) 
      {
        right = ONE_WHEEL_ROTATE_SPEED;
      } 
      else if (alpha < (360+ANGLE_TOLERANCE)) 
      {
        left = ONE_WHEEL_ROTATE_SPEED;
      }  
    }
    else //jade po -z
    {
      alpha = getAlpha(axis_x, -axis_z);
      
      //korekta ruchu
      if (alpha > ANGLE_TOLERANCE && alpha < 180) 
      {
        right = ONE_WHEEL_ROTATE_SPEED;
      } 
      else if (alpha < (360+ANGLE_TOLERANCE)) 
      {
        left = ONE_WHEEL_ROTATE_SPEED;
      }  
    }
  }

  left_speed = left;
  right_speed = right;
}

//losowanie wyboru kierunku jazdy
int randStep()
{
  int tab[3] = {0, 0, 0};
  int number = 0;
  int value = 0;

  if (!center_obstacle)
  {
    tab[number] = 1;
    ++number;
  }
  if (!right_obstacle)
  {
    tab[number] = 2;
    ++number;
  }
  if (!left_obstacle)
  {
    tab[number] = 3;
    ++number;
  }

  if (number > 1)
  {
    srand(time(NULL));
    value = (rand() % (number));
  }

  return tab[value];
}

//określenie współrzędnych kolejnego punktu jazdy
void getNextStep(double northx, double northz)
{
  target = false;  

  if (TYPE == 1)
  {
    //wykrycie możlowości
    if(!center_obstacle || !right_obstacle || !left_obstacle)
    {
      int randValue = randStep(); //los kierunku jazdy
  
      if (randValue == 1) //jade naprzód
      {
        if (go_x) //jade po x
        {
          if (target_axis_x == 1.0) //jade po +x
          {
            target_x += SIZE_STEP/2;  
          }
          else //jade po -x
          {
            target_x -= SIZE_STEP/2;
          }
        }
        else //jade po z
        {
          if (target_axis_z == 1.0) //jade po +z
          {
            target_z += SIZE_STEP/2;
          }
          else //jade po -z
          {
            target_z -= SIZE_STEP/2;
          }
        }
      }
      else if (randValue == 2) //jade w prawo
      {
        turn_right = true;
  
        if (go_x) //jade po x
        {
          if (target_axis_x == 1.0) //jade po +x
          {
            target_z += SIZE_STEP/2;
            target_axis_z = 1.0; //obróc na +z
          }
          else //jade po -x
          {
            target_z -= SIZE_STEP/2;
            target_axis_z = -1.0; //obróc na -z
          }
  
          turnRight(northx); //skręt w prawo
        }
        else //jade po z
        {
          if (target_axis_z == 1.0) //jade po +z
          {
            target_x -= SIZE_STEP/2;
            target_axis_x = -1.0; //obróc na -x
          }
          else //jade po -z
          {
            target_x += SIZE_STEP/2;
            target_axis_x = 1.0; //obróc na +x
          }
      
          turnRight(northz); //skręt w prawo
        }
  
        go_x = !go_x; //zmiana osi ruchu
      }
      else //jade w lewo
      {
        turn_left = true;
  
        if (go_x) //jade po x
        {
          if (target_axis_x == 1.0) //jade po +x
          {
            target_z -= SIZE_STEP/2;
            target_axis_z = -1.0; //obróc na -z
          }
          else //jade po -x
          {
            target_z += SIZE_STEP/2;
            target_axis_z = 1.0; //obróc na +z
          }
      
          turnLeft(northx); //skręt w lewo
        }
        else //jade po z
        {
          if (target_axis_z == 1.0) //jade po +z
          {
            target_x += SIZE_STEP/2;
            target_axis_x = 1.0; //obróc na +x
          }
          else //jade po -z
          {
            target_x -= SIZE_STEP/2;
            target_axis_x = -1.0; //obróc na -x
          }
      
          turnLeft(northz); //skręt w lewo
        }
  
        go_x = !go_x; //zmiana osi ruchu
      }
    }
    else //brak możliwości ruchu, zawracam
    {
      turn_back = true;
  
      if (go_x) //jade po x
      {
        if (target_axis_x == 1.0) //jade po +x
        {
          target_x -= SIZE_STEP/2;
          target_axis_x = -1.0; //obróc na -x
        }
        else //jade po -x
        {
          target_x += SIZE_STEP/2;
          target_axis_x = 1.0; //obróc na +x
        }
  
        turnBack(northz); //zawracanie
      }
      else //jade po z
      {
        if (target_axis_z == 1.0) //jade po +z
        {
          target_z -= SIZE_STEP/2;
          target_axis_z = -1.0; //obróc na -z
        }
        else //jade po -z
        {
          target_z += SIZE_STEP/2;
          target_axis_z = 1.0; //obróc na +z
        }
  
        turnBack(northx); //zawracanie
      }
    }
  }
  else
  {
    if (road[step] == 1) //jade naprzód
    {
      if (go_x) //jade po x
      {
        if (target_axis_x == 1.0) //jade po +x
        {
          target_x += SIZE_STEP/2;  
        }
        else //jade po -x
        {
          target_x -= SIZE_STEP/2;
        }
      }
      else //jade po z
      {
        if (target_axis_z == 1.0) //jade po +z
        {
          target_z += SIZE_STEP/2;
        }
        else //jade po -z
        {
          target_z -= SIZE_STEP/2;
        }
      }
    }
    else if (road[step] == 2) //jade w prawo
    {
      turn_right = true;

      if (go_x) //jade po x
      {
        if (target_axis_x == 1.0) //jade po +x
        {
          target_z += SIZE_STEP/2;
          target_axis_z = 1.0; //obróc na +z
        }
        else //jade po -x
        {
          target_z -= SIZE_STEP/2;
          target_axis_z = -1.0; //obróc na -z
        }

        turnRight(northx); //skręt w prawo
      }
      else //jade po z
      {
        if (target_axis_z == 1.0) //jade po +z
        {
          target_x -= SIZE_STEP/2;
          target_axis_x = -1.0; //obróc na -x
        }
        else //jade po -z
        {
          target_x += SIZE_STEP/2;
          target_axis_x = 1.0; //obróc na +x
        }
     
        turnRight(northz); //skręt w prawo
      }

      go_x = !go_x; //zmiana osi ruchu
    }
    else //jade w lewo
    {
      turn_left = true;

      if (go_x) //jade po x
      {
        if (target_axis_x == 1.0) //jade po +x
        {
          target_z -= SIZE_STEP/2;
          target_axis_z = -1.0; //obróc na -z
        }
        else //jade po -x
        {
          target_z += SIZE_STEP/2;
          target_axis_z = 1.0; //obróc na +z
        }
    
        turnLeft(northx); //skręt w lewo
      }
      else //jade po z
      {
        if (target_axis_z == 1.0) //jade po +z
        {
          target_x += SIZE_STEP/2;
          target_axis_x = 1.0; //obróc na +x
        }
        else //jade po -z
        {
          target_x -= SIZE_STEP/2;
          target_axis_x = -1.0; //obróc na -x
        }
    
        turnLeft(northz); //skręt w lewo
      }
 
      go_x = !go_x; //zmiana osi ruchu
    }
    
    ++step;
  }
}

//sprawdzenie czy dotarłem do celu
void endCondition(double distance)
{
  //sprawdzenie czy cel osiągnięty
  if (distance < DISTANCE_TOLERANCE)
  {
    left_speed = 0;
    right_speed = 0;
    target = true;
        
    //jeżeli to cel końcowy
    if (TYPE == 1 && points == 0)
    {
      end = true;
      
      //zapis mapy do pliku
      FILE *file;
      file = fopen(PLIK, "w");
      
      for (int i=0; i<256; ++i)
      {
        fprintf(file, "%d\n", map[i]);
      }
      
      fclose(file);
     
      printf("MAPOWANIE ZAKOŃCZONE!");
    }
    else
    {
      if(target_x >= (END_TARGET_X - END_TARGET_TOLERANCE) && 
        target_x <= (END_TARGET_X + END_TARGET_TOLERANCE) &&
        target_z >= (END_TARGET_Z - END_TARGET_TOLERANCE) && 
        target_z <= (END_TARGET_Z + END_TARGET_TOLERANCE)
      )
      {
        end = true;
        printf("DOTARŁEM DO CELU!");
      }
    }
  }
}

//tworzenie mapy labiryntu
void mapping()
{
  if (go_x) //jade po x
  {
    if (target_axis_x == 1.0) //jade po +x
    {
      if (map[index] == 0)
      {
        if (!center_obstacle)
        {
          map[index] += WEST;
        }
        
        if (!right_obstacle)
        {
          map[index] += SOUTH;
        }
        
        if (!left_obstacle)
        {
          map[index] += NORTH;
        }
        
        map[index] += EAST;
        --points;
      }
      
      ++index;
    }
    else //jade po -x
    {
      if (map[index] == 0)
      {
        if (!center_obstacle)
        {
          map[index] += EAST;
        }
             
        if (!right_obstacle)
        {
          map[index] += NORTH;
        }
        
        if (!left_obstacle)
        {
          map[index] += SOUTH;
        }
        
        map[index] += WEST;
        --points;
      }
      
      --index;
    }
  }
  else //jade po z
  {
    if (target_axis_z == 1.0) //jade po +z
    {
      if (map[index] == 0)
      {
        if (!center_obstacle)
        {
          map[index] += SOUTH;
        }
        
        if (!right_obstacle)
        {
          map[index] += WEST;
        }
        
        if (!left_obstacle)
        {
          map[index] += EAST;
        }

        map[index] += NORTH;
        --points;
      }
      
      index -= 16;
    }
    else //jade po -z
    {
      if (map[index] == 0)
      {
        if (!center_obstacle)
        {
          map[index] += NORTH;
        }
        
        if (!right_obstacle)
        {
          map[index] += EAST;
        }
        
        if (!left_obstacle)
        {
          map[index] += WEST;
        }
        
        if (index != 0)
        {
          map[index] += SOUTH;
        }
        
        --points;
      }
   
      index += 16;
    }
  }
}

//tworzenie grafu
struct Graph* createGraph(struct Edge edges[], int n)
{ 
  int i;
  struct Graph* graph = (struct Graph*)malloc(sizeof(struct Graph));
  
  for(i=0; i<GRAFN; ++i)
  {
    graph->head[i] = NULL;
  }
  
  for(i=0; i<n; ++i)
  {
    int src = edges[i].src;
    int dest = edges[i].dest;
    int val = edges[i].val;
  	
    struct Node* newNode = (struct Node*)malloc(sizeof(struct Node));
  	
    newNode->dest = dest;
    newNode->val = val;
    newNode->next = graph->head[src];
    graph->head[src] = newNode;
  }
  
  return graph;
}

//przygotowanie grafu
void prepareGraph()
{
  //odczytanie danych z pliku
  FILE *file;
  file = fopen(PLIK, "r");
  int i;    
	      
  for(i=0; i<GRAFN; ++i)
  fscanf(file, "%d", &map[i]);
	      
  fclose(file);

  //tworzenie spisu wierchołków grafu
  int x=0;
  struct Edge edges[EDGEN];

  for (i=0; i<GRAFN; ++i)
  {
    int val = map[i];

    if (val >= NORTH)
    {
      val -= NORTH;
      edges[x].src = i;
      edges[x].val = i;
      edges[x++].dest = i+16;
    }
	
    if (val >= EAST)
    {
      val -= EAST;
      edges[x].src = i;
      edges[x].val = i;
      edges[x++].dest = i+1;
    }
  	
    if (val >= SOUTH)
    {
      val -= SOUTH;
      edges[x].src = i;
      edges[x].val = i;
      edges[x++].dest = i-16;
    }
	
    if (val >= WEST)
    {
      val -= WEST;
      edges[x].src = i;
      edges[x].val = i;
      edges[x++].dest = i-1;
    }
  }

  //stworzenie grafu
  graph = createGraph(edges, EDGEN);
}

//propagacja fali
void wavePropagation()
{
  int i, j;
  int table[GRAFN];
  int elem = 10;
  int visited[GRAFN];
  int listA[elem];
  int listN[elem];
  int countA = 1;
  int countN = 0;
  int val = 0;
  bool end = true;
  int active = ENDGRAPH;
  int last = 3;
  
  for(i=0; i<GRAFN; ++i)
  {
    visited[i] = 0;
    table[i] = GRAFN;
  }
  
  for(i=0; i<elem; ++i)
  {
    listA[i] = 0;
    listN[i] = 0;
  }
  
  while (end)
  {
    for (i=0; i<countA; ++i)
    { 
      visited[listA[i]] = 1;
  	
      struct Node* head = graph->head[listA[i]];
  	
      while(head != NULL)
      {
        table[listA[i]] = val;
	  	
        if (visited[head->dest] == 0)
        {
          bool check = true;
          
          for (j=0; j<countN; ++j)
          {
            if (listN[j] == head->dest)
            {
              check = false;	
            }
          }	
          
          if (check)
          {
            listN[countN++] = head->dest;
          }	
        }
        
        head = head->next;
      }
    }
    
    ++val;
    countA = countN;
    countN = 0;
    
    for (i=0; i<elem; ++i)
    {
      listA[i] = listN[i];
      listN[i] = 0;
    }
	
    bool check = true;
	
    for (i=0; i<GRAFN; ++i)
    {
      if (visited[i] == 0)
      {
        check = false;
        break;
      }
    }
      
    if (check)
    {
      end = false;
    }
  }
  
  end = true;
  val = table[active];
  road[0] = ENDGRAPH;
  countR = 1;
  
  while (end)
  {
    bool left = false, right = false, up = false, down = false;
  
    if (active > 15 && table[active-16] == val-1)
    {
      struct Node* ptr = graph->head[active-16];
      
      while(ptr != NULL)
      {
        if (ptr->dest == active)
        {
          down = true;
          break;
        }
        
        ptr = ptr->next;
      }
    }
    
    if (active%16 != 0 && table[active-1] == val-1)
    {
      struct Node* ptr = graph->head[active-1];
      
      while(ptr != NULL)
      {
        if (ptr->dest == active)
        {
          left = true;
          break;
        }
        
        ptr = ptr->next;
      }
    }
    
    if (active < 240 && table[active+16] == val-1)
    {
      struct Node* ptr = graph->head[active+16];
      
      while(ptr != NULL)
      {
        if (ptr->dest == active)
        {
          up = true;
          break;
        }
        
        ptr = ptr->next;
      }
    }
    
    if ((active+1)%16 != 0 && table[active+1] == val-1)
    {
      struct Node* ptr = graph->head[active+1];
      
      while(ptr != NULL)
      {
        if (ptr->dest == active)
        {
          right = true;
          break;
        }
        ptr = ptr->next;
      }
    }
    
    if (last == 1 && down)
    {
      active -= 16;
    }
    else if (last == 2 && left)
    {
      --active;
    }
    else if (last == 3 && up)
    {
      active += 16;
    }
    else if (last == 4 && right)
    {
      ++active;
    }
    else if (down)
    {
      active -= 16;
      last = 1;
    }
    else if (left)
    {
      --active;
      last = 2;
    }
    else if (up)
    {
      active += 16;
      last = 3;
    }
    else if (right)
    {
      ++active;
      last = 4;
    }
    
    road[countR++] = active;
    --val;
    
    if (active == 0)
    {
      end = false;
    } 
  }
}  

//optymalizacja drogi
void correctWay()
{  
  int i;
  int tmp[--countR];
  int dir = 1;
  int active = 0;
  int val = 0;
  
  for (i=countR-1; i>=0; --i)
  {
    //góra
    if (dir == 1)
    {
      if (road[i] == active+16)
      {
        tmp[val++] = 1;
      }
      else if (road[i] == active+1)
      {
        tmp[val++] = 2;
        dir = 2;
      }
      else
      {
        tmp[val++] = 3;
        dir = 3;
      }
    }
    //prawo
    else if (dir == 2)
    {
      if (road[i] == active+1)
      {
        tmp[val++] = 1;
      }
      else if (road[i] == active-16)
      {
        tmp[val++] = 2;
        dir = 4;
      }
      else
      {
        tmp[val++] = 3;
        dir = 1;
      }
    }
    //lewo
    else if (dir == 3)
    {
      if (road[i] == active-1)
      {
        tmp[val++] = 1;
      }
      else if (road[i] == active+16)
      {
        tmp[val++] = 2;
        dir = 1;
      }
      else
      {
        tmp[val++] = 3;
        dir = 4;
      }
    }
    //dół
    else
    {
      if (road[i] == active-16)
      {
        tmp[val++] = 1;
      }
      else if (road[i] == active-1)
      {
        tmp[val++] = 2;
        dir = 3;
      }
      else
      {
        tmp[val++] = 3;
        dir = 2;
      }
    }
    
    active = road[i];
  }
  
  for(i=0; i<100; ++i)
  {
    road[i] = 0;
  }
  
  for(i=1; i<countR; ++i)
  {
    road[i-1] = tmp[i];
  }
}

//algorytm Dijkstry
void algorithmDijkstra()
{
  int i;
  int d[GRAFN];
  int p[GRAFN];
  int elem = 100;
  int visited[GRAFN];
  int listA[elem];
  int listN[elem];
  int listP[elem];
  int listPN[elem];
  int countA = 1;
  int countN = 0;
  int val = 0;
  bool end = true;
  
  for(i=0; i<GRAFN; ++i)
  {
    visited[i] = 0;
    d[i] = GRAFN;
    p[i] = -1;
  }
  
  d[0] = 0;
  
  for(i=0; i<elem; ++i)
  {
    listA[i] = 0;
    listN[i] = 0;
    listP[i] = 0;
    listPN[i] = 0;
  }
  
  while (end)
  {
    for (i=0; i<countA; ++i)
    { 
      visited[listA[i]] = 1;
      
      bool checkWay = false;
      
      if (listA[i] != 0)
      {
        int cost = 1;
        
        if (listA[i] != 16 && (abs(listA[i] - listP[i]) != abs(listP[i] - p[listP[i]])))
        {
          cost = 3;	
        }
        
        if (d[listA[i]] > d[listP[i]] + cost)
        {
          d[listA[i]] = d[listP[i]] + cost;
          p[listA[i]] = listP[i];
          
          checkWay = true;
        }
      }
      
      struct Node* head = graph->head[listA[i]];
      
      while(head != NULL)
      {
        if (visited[head->dest] == 0 || checkWay)
        {
          bool check = true;
          
          if (check)
          {
            listPN[countN] = listA[i];
            listN[countN++] = head->dest;
          }
        }
          
        head = head->next;
      }
    } 
    
    countA = countN;
    countN = 0;
    
    for (i=0; i<elem; ++i)
    {
      listP[i] = listPN[i];
      listPN[i] = 0;
      listA[i] = listN[i];
      listN[i] = 0;
    }
    
    bool check = true;
	
    for (i=0; i<GRAFN; ++i)
    {
      if (visited[i] == 0)
      {
        check = false;
        break;
      }
    }
      
    if (check)
    {
      end = false;
    }
  }
  
  val = ENDGRAPH;
  countR = 0;
  
  while (p[val] != -1)
  {
    road[countR++] = p[val];
    val = p[val];
  }
}

int main() 
{
  //inicjalizacja robota
  wb_robot_init();

  //tagi silnika
  WbDeviceTag leftMotor = wb_robot_get_device("left wheel motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right wheel motor");

  //tagi sensorów
  WbDeviceTag centralRightSensor = wb_robot_get_device("ps0");
  WbDeviceTag centralLeftSensor = wb_robot_get_device("ps7");
  WbDeviceTag rightSensor = wb_robot_get_device("ps2");
  WbDeviceTag leftSensor = wb_robot_get_device("ps5");

  //włączenie sensorów
  wb_distance_sensor_enable(centralRightSensor, TIME_STEP);
  wb_distance_sensor_enable(centralLeftSensor, TIME_STEP);
  wb_distance_sensor_enable(leftSensor, TIME_STEP);
  wb_distance_sensor_enable(rightSensor, TIME_STEP);

  //ustawienie pozycji silników
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  
  //ustawienie kompasu
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  //ustawienie gps
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  if (TYPE == 1)
  {
    for (int i=0; i<256; ++i)
    {
      map[i] = 0;
    }
    
    map[0] = 8;
   }
   else
   {
     //stworzenie grafu na podstawie danych z pliku
     prepareGraph();
     
     if (TYPE == 2)
     {
       //uruchomienie algorytmu propagacji fali
       wavePropagation();
     }
     else 
     {
       //uruchomienie algorytmu Dijkstry
       algorithmDijkstra();
     }
     
     //przygotowanie drogi do odczytu przez robota
     correctWay();
   }

  while (wb_robot_step(TIME_STEP) != -1) 
  {
    //pobranie wartości sensorów
    double centralLeftSensorValue = wb_distance_sensor_get_value(centralLeftSensor);
    double centralRightSensorValue = wb_distance_sensor_get_value(centralRightSensor);
    double leftSensorValue = wb_distance_sensor_get_value(leftSensor);
    double rightSensorValue = wb_distance_sensor_get_value(rightSensor);
    
    //wykrycie obiektów kolizji
    center_obstacle = centralLeftSensorValue > OBSTACLE_VALUE || centralRightSensorValue > OBSTACLE_VALUE;
    right_obstacle = rightSensorValue > OBSTACLE_VALUE;
    left_obstacle = leftSensorValue > OBSTACLE_VALUE;
    
    //pobranie współrzędnych (x,z) kompasu
    const double northx = wb_compass_get_values(compass)[0];
    const double northz = wb_compass_get_values(compass)[2];  
    
    //pobranie współrzędnych (x,z) gps
    const double gpsxx = wb_gps_get_values(gps)[0];
    const double gpszz = wb_gps_get_values(gps)[2];

    double gpsx = gpsxx;
    double gpsz = gpszz;
    if (go_x)
    {
      gpsx += -target_axis_x * 0.03;
    }
    else
    {
      gpsz += -target_axis_z * 0.034018;
    }

    //definicja wektora odległości (od robota do celu)
    double x1 = target_x - gpsx;
    double z1 = target_z - gpsz;
    
    //określenie odległości do celu
    double distance = sqrt(x1*x1 + z1*z1);  
    
    //zachowanie robota
    if (turn_left) //czy skręcam w lewo
    {
      if (go_x) //jade po x
      {
        turnLeft(northz); //skręt w lewo
      }
      else //jade po z
      {
        turnLeft(northx); //skręt w lewo
      }
    }
    else if (turn_right) //czy skręcam w prawo
    {
      if (go_x) //jade po x
      {
        turnRight(northz); //skręt w prawo
      }
      else //jade po z
      {
        turnRight(northx); //skręt w prawo
      }
    }
    else if (turn_back) //czy zawracam
    {
      if (go_x) //jade po x
      {
        turnBack(northz); //zawracanie
      }
      else //jade po z
      {
        turnBack(northx); //zawracanie
      }
    }
    else if (!target) //jeżeli nie ma celu to jadę dalej
    {   
      goStraight(northx, northz); //jazda naprzód
      endCondition(distance); //sprawdzenie czy dotarłem do celu
    }
    else if (!end) //jeżeli jeszcze nie dotarłem do końca
    {    
      getNextStep(northx, northz); //okrelenie kolejnego celu jazdy
      
      if (TYPE == 1)
      {
        mapping();
      }
    }

    //ustawienie szybkości silników 
    wb_motor_set_velocity(leftMotor, left_speed);
    wb_motor_set_velocity(rightMotor, right_speed);
  }
  
  wb_robot_cleanup();

  return 0;
}