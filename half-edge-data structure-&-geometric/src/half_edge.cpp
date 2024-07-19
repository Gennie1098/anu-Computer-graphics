#include <include/half_edge.hpp>
#include <Eigen/Dense>

/*
#################################################################################
#                       Vertex-related Helper Functions                         #
#################################################################################
*/

// Optinal TODO: Iterate through all neighbour vertices around the vertex
// Helpful when you implement the average degree computation of the mesh
std::vector<std::shared_ptr<Vertex>> Vertex::neighbor_vertices() {
    // std::vector<std::shared_ptr<Vertex>> neighborhood;
    // return neighborhood; 

    std::vector<std::shared_ptr<Vertex>> neighborhood;
    std::shared_ptr<HalfEdge> start = this->he, current = this->he;
    if (!start) {
        std::cout << "No starting half-edge for vertex ID: " << this->id << std::endl;
        return neighborhood;  
    }
    do {
        if (current && current->twin && current->twin->vertex) {
            neighborhood.push_back(current->twin->vertex);
            std::cout << "Neighbor vertex ID: " << current->twin->vertex->id << " for vertex ID: " << this->id << std::endl;
            current = current->twin->next;
        } else {
            break;
        }
    } while (current != start && current);
    return neighborhood;
}


// TODO: Iterate through all half edges pointing away from the vertex
std::vector<std::shared_ptr<HalfEdge>> Vertex::neighbor_half_edges() {
    // std::vector<std::shared_ptr<HalfEdge>> neighborhood;
    // return neighborhood;

    std::vector<std::shared_ptr<HalfEdge>> neighborhood;
    if (!this->he) {
        std::cout << "No starting half-edge for vertex ID: " << this->id << std::endl;
        return neighborhood;  // Return empty list if no half-edge exists
    }

    std::shared_ptr<HalfEdge> start_he = this->he;  // Get the starting half-edge
    std::shared_ptr<HalfEdge> current_he = start_he;
    do {
        if (!current_he || !current_he->twin) {
            std::cout << "Broken half-edge chain at vertex ID: " << this->id << std::endl;
            break;  // Break if twin is null to prevent nullptr dereference
        }
        neighborhood.push_back(current_he);  // Add the current half-edge to the list
        current_he = current_he->twin->next;  // Move to the next half-edge pointing away from the vertex
    } while (current_he && current_he != start_he);  // Loop until back to the start

    return neighborhood;  // Return the list of half-edges
}


// TODO: Computate quadratic error metrics coefficient, which is a 5-d vector associated with each vertex
/*
    HINT:
        Please refer to homework description about how to calculate each element in the vector.
        The final results is stored in class variable "this->qem_coff"
*/
void Vertex::compute_qem_coeff() {
    // this->qem_coff = Eigen::VectorXf(5);

    std::vector<std::shared_ptr<Vertex>> neighbors = this->neighbor_vertices();  // Get list of neighbor vertices
    int n = neighbors.size(); 
    Eigen::Vector3f sum_vi(0.0, 0.0, 0.0);  // Sum of neighbor vertices's postion
    float sum_vivi = 0.0;  

    for (const auto& neighbor : neighbors) {
        sum_vi += neighbor->pos;  
        sum_vivi += neighbor->pos.dot(neighbor->pos);  
    }
    Eigen::VectorXf qem(5);
    qem << n, sum_vi(0), sum_vi(1), sum_vi(2), sum_vivi;  
    this->qem_coff = qem;  // Update QEM
}


/*
#################################################################################
#                         Face-related Helper Functions                         #
#################################################################################
*/

// TODO: Iterate through all member vertices of the face
std::vector<std::shared_ptr<Vertex>> Face::vertices() {
    // std::vector<std::shared_ptr<Vertex>> member_vertices;
    // return member_vertices;

    std::vector<std::shared_ptr<Vertex>> member_vertices;
    if (!this->he) return member_vertices;  // Chedk if face has he

    std::shared_ptr<HalfEdge> start = this->he, current = this->he;
    do {
        if (current && current->vertex) {
            member_vertices.push_back(current->vertex);  // add vertex to list
            current = current->next;  // move to the next he
        } else {
            break;  
        }
    } while (current != start && current);  // stop when go back to the start or null

    return member_vertices;
}


// TODO: implement this function to compute the area of the triangular face
float Face::get_area(){
    float area;

    return area;
}

// TODO: implement this function to compute the signed volume of the triangular face
// reference: http://chenlab.ece.cornell.edu/Publication/Cha/icip01_Cha.pdf eq.(5)
float Face::get_signed_volume(){
    float volume;

    return volume;
}


/*
#################################################################################
#                         Edge-related Helper Functions                         #
#################################################################################
*/

/*
    TODO: 
        Compute the contraction information for the edge (v1, v2), which will be used later to perform edge collapse
            (i) The optimal contraction target v*
            (ii) The quadratic error metrics QEM, which will become the cost of contracting this edge
        The final results is stored in class variable "this->verts_contract_pos" and "this->qem"
    Please refer to homework description for more details
*/


void Edge::compute_contraction() {
    // this->verts_contract_pos = Eigen::Vector3f(0, 0, 0);
    // this->qem = 0;

    // Đảm bảo rằng half-edge có đỉnh và twin của nó cũng có đỉnh
    if (!this->he || !this->he->vertex || !this->he->twin || !this->he->twin->vertex) {
        std::cout << "Edge contraction precondition failed: vertices missing." << std::endl;
        return;
    }

    // 2 vertices of edge
    auto v1 = this->he->vertex;
    auto v2 = this->he->twin->vertex;

    auto q1 = v1->qem_coff;
    auto q2 = v2->qem_coff;

    // Calculate the midpoint as the optimal collapsing position
    this->verts_contract_pos = (v1->pos + v2->pos) * 0.5;

    // q* = q1 + q2
    Eigen::VectorXf qem_total = v1->qem_coff + v2->qem_coff;

    Eigen::Vector3f v_star = verts_contract_pos; // v*

    // Vector [v^Tv, -2v, 1]
    Eigen::VectorXf contraction_vector(5);
    contraction_vector << v_star.squaredNorm(), -2.0 * v_star, 1.0;

    // Cal qem_error = q_total * contraction_vector
    float qem_error = qem_total.dot(contraction_vector);

    // Update the QEM error for this edge
    this->qem = qem_error;

    std::cout << "Computed contraction for edge " << this->id << ": Position = " << this->verts_contract_pos.transpose() << ", QEM = " << this->qem << std::endl;
}

/*
    TODO: 
        Perform edge contraction functionality, which we write as (v1 ,v2) → v*, 
            (i) Moves the vertex v1 to the new position v*, remember to update all corresponding attributes,
            (ii) Connects all incident edges of v1 and v2 to v*, and remove the vertex v2,
            (iii) All faces, half edges, and edges associated with this collapse edge will be removed.
    HINT: 
        (i) Pointer reassignments
        (ii) When you want to remove mesh components, simply set their "exists" attribute to False
    Please refer to homework description for more details
*/
void Edge::edge_contraction() {

    // Label elements
    auto v1 = this->he->vertex;
    auto v2 = this->he->twin->vertex;

    auto face1 = this->he->face;
    auto face2 = this->he->twin->face;

    auto he1 = this->he;
    auto he2 = this->he->twin;
    auto edge1 = this;

    auto he3 = he2->next;
    auto he4 = he3->twin;
    auto edge2 = he3->edge;
    
    auto he5 = he3->next;
    auto he6 = he5->twin;
    auto edge3 = he5->edge;
    auto v3 = he5->vertex;

    auto he7 = he1->next;
    auto he8 = he7->twin;
    auto edge4 = he7->edge;

    auto he9 = he7->next;
    auto he10 = he9->twin;
    auto edge5 = he9->edge;
    auto v4 = he9->vertex;

    // V1 to new position
    v1->pos = this->verts_contract_pos;

    // Reassign all remain he from v2 to v1
    std::vector<std::shared_ptr<HalfEdge>> all_he_v2 = v2->neighbor_half_edges(); //all he associated with v2
    for (auto& he : all_he_v2) {
        if (he->exists) {  // only exist he
            he->vertex = v1;  // to v1
        }
    }

    // Remove v2
    v2->exists = false;

    // Remove edges and he
    he1->exists = false;
    he2->exists = false;
    edge1->exists = false;

    he3->exists = false;
    edge2->exists = false;

    he5->exists = false;

    he7->exists = false;
    edge4->exists = false;

    he9->exists = false;

    // Remove 2 faces
    face1->exists = false;
    face2->exists = false;

    // Reassign edges and he to v1
    v1->he = he6;

    edge5->he = he10;
    he10->twin = he8;
    he8->twin = he10;
    he8->edge = edge5;

    edge3->he = he6;
    he6->vertex = v1;
    he6->twin = he4;

    he10->vertex = v1;

    // Reassign he to v1
    
    he6->edge = edge3;

    v3->he = he4;
    he4->twin = he6;
    he4->edge = edge3;

    v4->he = he8;
    he8->edge = edge5;
    he8->twin = he10;

    v1->qem_coff = v1->qem_coff + v2->qem_coff;

    std::cout << "Edge contraction performed on edge " << this->id << std::endl;
}
