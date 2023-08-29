import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

import java.lang.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

public class tsp {

    public static void main(String[] args){
        int numberOfPoints = 30;
        Integer[][] distanceMatrix = generateDistanceData(numberOfPoints);
        ArrayList<Object> valueAndTour = solveModel(distanceMatrix);
        System.out.println("Value: " + (double) valueAndTour.get(0));
        System.out.println("Tour: " + valueAndTour.get(1));
    }

    public static ArrayList<Object> solveModelEliminate(Integer[][] distanceMatrix, ArrayList<ArrayList<Integer>> Subtours){
        MPSolver solver = MPSolver.createSolver("SCIP");
        int n = distanceMatrix.length;
        double infinity = java.lang.Double.POSITIVE_INFINITY;

        // Create the variables
        MPVariable[][] x = new MPVariable[n][];
        for (int i=0; i < n; i++){
            for (int j=0; j < n; j++){
                if (i != j){
                    x[i][j] = solver.makeIntVar(0,1,"x"+i+","+j);
                }
                else{
                    x[i][j] = solver.makeIntVar(0,0,"x"+i+","+j);
                }
            }
        }

        // Define the constraints
        for (int i=0; i < n; i++){
            MPConstraint cNeg1 = solver.makeConstraint(-infinity,1, "cNeg1_"+i);
            MPConstraint cPos1 = solver.makeConstraint(infinity,1, "cPos1_"+i);
            for (int j=0; j < n; j++){
                cNeg1.setCoefficient(x[i][j], 1);
                cPos1.setCoefficient(x[i][j], 1);
            }

            MPConstraint cNeg2 = solver.makeConstraint(-infinity,1, "cNeg2_"+i);
            MPConstraint cPos2 = solver.makeConstraint(infinity,1, "cPos2_"+i);
            for (int j=0; j < n; j++){
                cNeg2.setCoefficient(x[j][i], 1);
                cPos2.setCoefficient(x[j][i], 1);
            }

            MPConstraint cNeg3 = solver.makeConstraint(-infinity,0, "cNeg3_"+i);
            MPConstraint cPos3 = solver.makeConstraint(infinity,0, "cPos3_"+i);
            cNeg3.setCoefficient(x[i][i],1);
            cPos3.setCoefficient(x[i][i],1);
        }

        // Define and add Subtour constraints
        for (ArrayList<Integer> sub : Subtours){
            MPConstraint cSub = solver.makeConstraint(-infinity,sub.size()-1);
            for (int i=0; i < sub.size()-1; i++){
                for (int j=i+1; j < sub.size(); j++){
                    cSub.setCoefficient(x[sub.get(i)][sub.get(j)],1);
                    cSub.setCoefficient(x[sub.get(j)][sub.get(i)],1);
                }
            }
        }

        // Define the objective function
        MPObjective objective = solver.objective();
        for (int i=0; i < n; i++){
            for (int j=0; j<n; j++){
                if (distanceMatrix[i][j] != null) {
                    objective.setCoefficient(x[i][j], distanceMatrix[i][j]);
                }
                else{
                    objective.setCoefficient(x[i][j],0);
                }
            }
        }
        objective.setMinimization();

        // Invoke the solver
        MPSolver.ResultStatus resultStatus = solver.solve();
        ArrayList<ArrayList<Integer>> tours = extractTours(SolVal(x), n);

        ArrayList<Object> resultsValueTours = new ArrayList<>();
        resultsValueTours.add(resultStatus);
        resultsValueTours.add(objective.value());
        resultsValueTours.add(tours);
        return resultsValueTours;
    }

    public static ArrayList<ArrayList<Integer>> extractTours(ArrayList<ArrayList<Integer>> R, int n){
        ArrayList<ArrayList<Integer>> tours = new ArrayList<>();
        tours.add(new ArrayList<>(Collections.singletonList(0)));
        tours.get(0).add(0);
        int node = 0;

        ArrayList<Integer> allNodes = new ArrayList<>(Collections.nCopies(n, 1));
        allNodes.set(0,0);

        boolean sumAllNodesPositive = true;
        while (sumAllNodesPositive){

            ArrayList<Integer> nextP = new ArrayList<>();
            for (int i=0; i<n; i++){
                if (R.get(node).get(i) == 1){
                    nextP.add(i);
                }
            }

            int next = nextP.get(0);
            if (tours.get(tours.size()-1).contains(next)){
                node = allNodes.indexOf(1);
                tours.add(new ArrayList<>(node));
            } else {
                tours.get(tours.size()-1).add(next);
                node = next;
            }
            allNodes.set(node,0);

            sumAllNodesPositive = false;
            for (int nodeVal : allNodes){
                if (nodeVal > 1){
                    sumAllNodesPositive = true;
                }
            }
        }

        return tours;
    }

    public static ArrayList<Object> solveModel(Integer[][] DistanceMatrix){
        ArrayList<ArrayList<Integer>> subtours = new ArrayList<>();
        ArrayList<ArrayList<Integer>> tours = new ArrayList<>();
        ArrayList<Object> resultsValueTours = new ArrayList<>();
        MPSolver.ResultStatus resultStatus;
        while (tours.size() != 1) {
            resultsValueTours = solveModelEliminate(DistanceMatrix, subtours);
            resultStatus = ((MPSolver.ResultStatus) resultsValueTours.get(0));
            if (resultStatus.swigValue() == 0) { //TODO: Confirm swigValue meaning
                tours = (ArrayList<ArrayList<Integer>>) resultsValueTours.get(2);
                subtours.addAll(tours);
            }
        }
        ArrayList<Object> valueAndTour = new ArrayList<>();
        valueAndTour.add(resultsValueTours.get(1));
        valueAndTour.add(tours.get(0));
        return valueAndTour;
    }

    public static Integer[][] generateDistanceData(int n){
        /**
        Example return of:     `R,points=gen_data(5)`

        R:
            [[None, 365, 633, 486, 378],
             [318, None, 973, 794, 245],
             [540, 908, None, 395, 929],
             [500, 792, 368, None, 534],
             [400, 264, 859, 603, None]]

        points:
            [(48, 64), (29, 92), (76, 9), (33, 17), (6, 74)]
        **/

        Random rand = new Random(0);
        Integer[][] points = new Integer[n][];
        for (int i=0; i<n; i++){
            points[i] = new Integer[n];
            points[i][0] = rand.nextInt(100) + 1;
            points[i][1] = rand.nextInt(100) + 1;
        }

        Integer[][] distanceMatrix = new Integer[n][];
        for (int i=0; i < n; i++){
            distanceMatrix[i] = new Integer[n];
            for (int j=0; j < n; j++){
                double perturb = rand.nextDouble() + 0.2;
                if (i != j){
                    distanceMatrix[i][j] = (int) (distance(points[i], points[j]) * perturb);
                }
                else{
                    distanceMatrix[i][j] = null;
                }
            }
        }

        return distanceMatrix;
    }

    public static double distance(Integer[] p1, Integer[] p2){
        return Math.round(10*Math.sqrt(Math.pow(p1[0]-p2[0],2) + Math.pow(p1[1]-p2[1],2)));
    }

    public static ArrayList<ArrayList<Integer>> SolVal(MPVariable[][] x) {
        ArrayList<ArrayList<Integer>> arr = new ArrayList<>();
        for (MPVariable[] xr : x) {
            arr.add(SolVal(xr));
        }
        return arr;
    }

    public static ArrayList<Integer> SolVal(MPVariable[] x) {
        ArrayList<Integer> arr = new ArrayList<>();
        for (MPVariable xv : x) {
            arr.add(SolVal(xv));
        }
        return arr;
    }

    public static Integer SolVal(MPVariable x) {
        if (x == null){
            return 0;
        }
        else{
            return (int) x.solutionValue();
        }
    }

}
