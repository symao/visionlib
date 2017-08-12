#include <slam/ray_tracing.h>
#include <vector>
#include <common/numeric.h>

namespace NJRobot
{
	struct GridLineTraversalLine{
		int     num_points;
		IntPoint*  points;
	};

	
	void gridLineCore( IntPoint start, IntPoint end, IntPointList& line )
	{
		int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;
		dx = abs(end.x-start.x); dy = abs(end.y-start.y);	
		line.clear();
		line.reserve(std::max(dx,dy));

		if (dy <= dx){
			d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
			//make the iteerator begin and end
			if (start.x > end.x){
				x = end.x; y = end.y;
				ydirflag = (-1);  //1:from start.y to end.y     -1:from end.y to start.y
				xend = start.x;
			}else{
				x = start.x; y = start.y;
				ydirflag = 1;
				xend = end.x;
			}
			line.push_back(IntPoint(x,y));//store the start point
			if (((end.y - start.y) * ydirflag) > 0){
				while (x < xend){
					x++;
					if (d <0){
						d+=incr1;
					}else{
						y++; d+=incr2;
					}
					line.push_back(IntPoint(x,y));
				}
			}else{
				while (x < xend){
					x++;
					if (d <0){
						d+=incr1;
					}else{
						y--; d+=incr2;
					}
					line.push_back(IntPoint(x,y));
				}
			}
		}else {//(dy > dx)
			d = 2*dx - dy;
			incr1 = 2*dx; incr2 = 2 * (dx - dy);
			if (start.y > end.y){
				y = end.y; x = end.x;
				yend = start.y;
				xdirflag = (-1);
			}else{
				y = start.y; x = start.x;
				yend = end.y;
				xdirflag = 1;
			}
			line.push_back(IntPoint(x,y));
			if (((end.x - start.x) * xdirflag) > 0){
				while (y < yend){
					y++;
					if (d <0){
						d+=incr1;
					}else{
						x++; d+=incr2;
					}
					line.push_back(IntPoint(x,y));
				}
			}else{
				while (y < yend){
					y++;
					if (d <0){
						d+=incr1;
					}else{
						x--; d+=incr2;
					}
					line.push_back(IntPoint(x,y));
				}
			}
		}
	}

	void gridLine( IntPoint start, IntPoint end,IntPointList& line ) {
		gridLineCore( start, end, line );
		//翻转直线，保证数据是从start到end的
		if ( start.x!=line[0].x || start.y!=line[0].y ) {
				int n = line.size();
				for(int i=0;i<n/2;i++){
					std::swap(line[i],line[n-1-i]);
				}
		}
	}
	
	IntPointList rayTrace( IntPoint start, IntPoint end )
	{
		IntPointList line;
		gridLine(start,end,line);
		return line;
	}

}

