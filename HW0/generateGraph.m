function Graph = generateGraph(n)

[r,c]=meshgrid(1:n,1:n);
Graph = [ r(:), c(:), rand(size(r(:))) ; 
          n+1 , n+2 , 0                  ];



end